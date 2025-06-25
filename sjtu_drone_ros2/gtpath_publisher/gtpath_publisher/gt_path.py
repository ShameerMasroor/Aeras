#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32
from math import sqrt

class PoseToPathNode(Node):
    def __init__(self):
        super().__init__('pose_to_path')

        # Declare and get parameters
        self.declare_parameter('pose_topic', '/simple_drone/gt_pose')
        self.declare_parameter('path_topic', '/gt_path')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('error_topic', 'error')
        self.declare_parameter('modified_path_topic', '/modified_path')
        self.declare_parameter('rmse_topic', '/rmse')

        gt_pose_topic = self.get_parameter('pose_topic').value
        path_topic = self.get_parameter('path_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        error_topic = self.get_parameter('error_topic').value
        modified_path_topic = self.get_parameter('modified_path_topic').value
        rmse_topic = self.get_parameter('rmse_topic').value

        # Initializing the Path message for ground truth path
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id

        #variables for rmse
        self.error_counter = 0
        self.error_sum = 0
        
        # Initializing ground truth pose
        self.gt_pose = None
        # Initializing a variable for the modified path
        self.modified_path = None

        # Creating publishers for Path message and error
        self.path_pub = self.create_publisher(Path, path_topic, 10)
        self.error_pub = self.create_publisher(Float32, error_topic, 10)  #error may have a float value. Keeping it 32 bits for storage efficiency

        #rmse publisher
        self.rmse_pub = self.create_publisher(Float32, rmse_topic, 10)

        # Subscribing to the modified path topic and ground truth pose topic
        self.modified_path_sub = self.create_subscription(Path, modified_path_topic, self.modified_path_callback, 10)
        self.pose_sub = self.create_subscription(Pose, gt_pose_topic, self.gt_pose_callback, 10)

        # Creating a timer to publish Path at 2 Hz
        self.timer = self.create_timer(0.5, self.publish_path_callback)

        

        self.get_logger().info(f"PoseToPathNode initialized with pose_topic='{gt_pose_topic}', path_topic='{path_topic}', error_topic='{error_topic}', and rmse_error = {rmse_topic} ")


    def gt_pose_callback(self, pose_msg):
        # Store the ground truth pose (gt_pose)
        self.gt_pose = pose_msg
        
        
        # Create a PoseStamped from the incoming Pose message
        pose_stamped = PoseStamped()# Initialize a variable for the modified path
        self.modified_path = None

        #the above message has two members - the header and the pose. The header holds frame and time info
        #the pose member holds the pose info

        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = self.frame_id
        pose_stamped.pose = pose_msg

        # Append the PoseStamped to the Path message
        self.path_msg.poses.append(pose_stamped)

    def publish_path_callback(self):
        # Update the Path header with the latest timestamp
        self.path_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the Path message
        self.path_pub.publish(self.path_msg)

    def modified_path_callback(self, modified_path_msg):
        # Store the modified path separately
        self.modified_path = modified_path_msg
        # self.get_logger().info("Received modified path message.")

        # Calculate and publish the error if gt_pose is available
        if self.gt_pose:   #checks if gt_pose is available or not
            error = self.calculate_error(self.gt_pose, modified_path_msg)
            rmse = self.calculate_RMSE()
            self.publish_error(error)
            self.publish_RMSE(rmse)
            
        else:
            self.get_logger().warn("gt_pose not available, cannot calculate error.")




    def calculate_error(self, gt_pose, path_msg):
        # Check if the path contains any poses
        if not path_msg.poses:
            self.get_logger().warn("Modified path has no poses. Cannot calculate error.")
            return float('inf')

        # Get the most recent pose from the modified path
        most_recent_pose = path_msg.poses[-1].pose
        most_recent_position = most_recent_pose.position
        gt_position = gt_pose.position #picks the most recent gt_pose

        # Calculating the Euclidean distance between the gt_pose and the most recent pose in the modified path
        distance = sqrt(
            (gt_position.x - most_recent_position.x)**2 +
            (gt_position.y - most_recent_position.y)**2 +
            (gt_position.z - most_recent_position.z)**2
        )

        self.error_sum+= distance**2  #accumulating the errors
        self.error_counter+=1  # this is n for rmse mean
        return distance


    def calculate_RMSE(self):
        rmse = sqrt((self.error_sum / self.error_counter))
        return rmse
        

    def publish_error(self, error):
        # Publish the calculated error
        error_msg = Float32()
        error_msg.data = error
        self.error_pub.publish(error_msg)
        # self.get_logger().info(f"Current error: {error}")

    def publish_RMSE(self, rmse):
    
        current_time = self.get_clock().now().to_msg()

        # Publish the calculated RMSE
        rmse_msg = Float32()
        rmse_msg.data = rmse
        self.rmse_pub.publish(rmse_msg)

        # Log RMSE with timestamp
        self.get_logger().info(f"Timestamp: {current_time.sec}.{current_time.nanosec}, Current RMSE: {rmse}")



def main(args=None):
    rclpy.init(args=args)
    node = PoseToPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("PoseToPathNode shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
