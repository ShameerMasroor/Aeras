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
        self.declare_parameter('distance_topic', '/distance_travelled')
        self.declare_parameter('modified_path_topic', '/modified_path')
        self.declare_parameter('rmse_topic', '/rmse')
        

        #for FASTLIO
        self.declare_parameter('lio_path', '/path')
        self.declare_parameter('lio_rmse', '/lio_rmse')

        #modification for glim
        self.declare_parameter('glim_rmse_topic', 'glim_rmse')
        self.declare_parameter('glim_pose', '/glim_ros/pose')
        self.declare_parameter('glim_path', 'glim_path')

        gt_pose_topic = self.get_parameter('pose_topic').value
        path_topic = self.get_parameter('path_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        error_topic = self.get_parameter('error_topic').value
        distance_topic = self.get_parameter('distance_topic').value
        modified_path_topic = self.get_parameter('modified_path_topic').value
        rmse_topic = self.get_parameter('rmse_topic').value

        #For FASTLIO
        lio_path = self.get_parameter('lio_path').value
        lio_rmse = self.get_parameter('lio_rmse').value

        #modification for GLIM
        glim_rmse = self.get_parameter('glim_rmse_topic').value
        glim_pose = self.get_parameter('glim_pose').value
        glim_path = self.get_parameter('glim_path').value
        #initializing glim path node
        self.glim_path = None
        self.glim_pose = None
        #for GLIM
        self.glim_path_pub = self.create_publisher(Path, glim_path, 10)
        self.glim_pose_sub = self.create_subscription(Pose, glim_pose, self.GLIM_path_callback, 10)
        self.glim_rmse_pub = self.create_publisher(Float32, glim_rmse, 10)

        #for FASTLIO
        self.create_subscription(Path, lio_path, self.FASTLIO_callback,10)
        self.fastlio_rmse_pub = self.create_publisher(Float32, lio_rmse, 10)

        # Initializing the Path message for ground truth path
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id

        # variables for rmse
        self.error_counter = 0
        self.error_sum = 0

        # error counters for glim
        self.glim_error_counter = 0
        self.glim_error_sum = 0

        #errors for FASTLIO
        self.fast_error_sum = 0
        self.fast_error_counter = 0
        
        # Initializing ground truth pose
        self.gt_pose = None
        self.prev_gt_pose = None
        self.total_distance = 0.0  # Initialize total distance traveled

        # Creating publishers for Path message, error, and distance traveled
        self.path_pub = self.create_publisher(Path, path_topic, 10)
        self.error_pub = self.create_publisher(Float32, error_topic, 10)
        self.distance_pub = self.create_publisher(Float32, distance_topic, 10)

        # RMSE publisher
        self.rmse_pub = self.create_publisher(Float32, rmse_topic, 10)

        # Subscribing to the modified path topic and ground truth pose topic
        self.modified_path_sub = self.create_subscription(Path, modified_path_topic, self.modified_path_callback, 10)
        self.pose_sub = self.create_subscription(Pose, gt_pose_topic, self.gt_pose_callback, 10)

        # Creating a timer to publish Path at 2 Hz
        self.timer = self.create_timer(0.5, self.publish_path_callback)

        self.get_logger().info(f"PoseToPathNode initialized with pose_topic='{gt_pose_topic}', path_topic='{path_topic}', error_topic='{error_topic}', distance_travelled_topic='{distance_topic}', li_rmse_error = {rmse_topic}, and GLIM RMSE = '{glim_rmse}', and FASTLIO RMSE = '{lio_rmse}' ")

    def gt_pose_callback(self, pose_msg):
        # Store the ground truth pose (gt_pose)
        self.gt_pose = pose_msg
        
        if self.prev_gt_pose:
            distance = self.calculate_distance_travelled(self.prev_gt_pose, self.gt_pose)
            self.total_distance += distance
            

        # Save the current pose as the previous one for the next callback
        self.prev_gt_pose = pose_msg

        # Create a PoseStamped from the incoming Pose message
        pose_stamped = PoseStamped()
        self.modified_path = None

        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = self.frame_id
        pose_stamped.pose = pose_msg

        # Append the PoseStamped to the Path message
        self.path_msg.poses.append(pose_stamped)

    def calculate_distance_travelled(self, prev_pose, current_pose):
        # Calculate Euclidean distance between two poses
        prev_position = prev_pose.position
        current_position = current_pose.position

        distance = sqrt(
            (current_position.x - prev_position.x)**2 +
            (current_position.y - prev_position.y)**2 +
            (current_position.z - prev_position.z)**2
        )
        return distance

    def publish_distance_travelled(self, total_distance):
        # Publish the calculated total distance traveled
        distance_msg = Float32()
        distance_msg.data = total_distance
        self.distance_pub.publish(distance_msg)
        self.get_logger().info(f"Total distance traveled: {total_distance} meters.")

    def publish_path_callback(self):
        # Update the Path header with the latest timestamp
        self.path_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the Path message
        self.path_pub.publish(self.path_msg)
        

    def modified_path_callback(self, modified_path_msg):
        # Store the modified path separately
        self.modified_path = modified_path_msg

        # Calculate and publish the error if gt_pose is available
        if self.gt_pose:   #checks if gt_pose is available or not
            error = self.calculate_error(self.gt_pose, modified_path_msg)
            rmse = self.calculate_RMSE()
            self.publish_error(error)
            self.publish_RMSE(rmse)
            self.publish_distance_travelled(self.total_distance)
        else:
            self.get_logger().warn("gt_pose not available, cannot calculate error.")

    def GLIM_path_callback(self, glim_pose_msg):
        # Create a PoseStamped from the incoming GLIM Pose message
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = self.frame_id
        pose_stamped.pose = glim_pose_msg

        # Initialize GLIM Path if it's None
        if self.glim_path is None:
            self.glim_path = Path()
            self.glim_path.header.frame_id = self.frame_id

        # Append the new pose to the GLIM Path
        self.glim_path.poses.append(pose_stamped)

        # Update the header timestamp
        self.glim_path.header.stamp = self.get_clock().now().to_msg()

        # Publish the GLIM Path
        self.glim_path_pub.publish(self.glim_path)

        # Log publishing event
        self.get_logger().info(f"Published GLIM path with {len(self.glim_path.poses)} poses.")

        # Calculate and publish the GLIM RMSE if ground truth pose is available
        if self.gt_pose:
            glim_rmse = self.calculate_GLIM_RMSE(self.gt_pose, glim_pose_msg)
            self.publish_GLIM_RMSE(glim_rmse)
        else:
            self.get_logger().warn("gt_pose not available, cannot calculate GLIM RMSE.")




    def FASTLIO_callback(self, lio_path_msg):
        # Store the modified path separately
        self.lio_path = lio_path_msg

        # Calculate and publish the error if gt_pose is available
        if self.gt_pose:   #checks if gt_pose is available or not
            fast_error = self.calculate_fast_error(self.gt_pose, lio_path_msg)
            lio_rmse = self.calculate_fast_RMSE()
            # self.publish_error(fast_error)
            self.publish_FASTLIO_RMSE(lio_rmse)
            self.publish_distance_travelled(self.total_distance)
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
        gt_position = gt_pose.position

        # Calculating the Euclidean distance between the gt_pose and the most recent pose in the modified path
        distance = sqrt(
            (gt_position.x - most_recent_position.x)**2 +
            (gt_position.y - most_recent_position.y)**2 +
            (gt_position.z - most_recent_position.z)**2
        )

        self.error_sum += distance**2  # accumulating the errors
        self.error_counter += 1  # this is n for RMSE mean
        return distance
    

    def calculate_fast_error(self, gt_pose, path_msg):
        # Check if the path contains any poses
        if not path_msg.poses:
            self.get_logger().warn("Modified path has no poses. Cannot calculate error.")
            return float('inf')

        # Get the most recent pose from the modified path
        most_recent_pose = path_msg.poses[-1].pose
        most_recent_position = most_recent_pose.position
        gt_position = gt_pose.position

        # Calculating the Euclidean distance between the gt_pose and the most recent pose in the modified path
        distance = sqrt(
            (gt_position.x - most_recent_position.x)**2 +
            (gt_position.y - most_recent_position.y)**2 +
            (gt_position.z - most_recent_position.z)**2
        )

        self.fast_error_sum += distance**2  # accumulating the errors
        self.fast_error_counter += 1  # this is n for RMSE mean
        return distance

    def calculate_RMSE(self):
        rmse = sqrt((self.error_sum / self.error_counter))
        return rmse
    
    def calculate_fast_RMSE(self):
        rmse = sqrt((self.fast_error_sum / self.fast_error_counter))
        return rmse

    def publish_error(self, error):
        # Publish the calculated error
        error_msg = Float32()
        error_msg.data = error
        self.error_pub.publish(error_msg)

        

    def publish_RMSE(self, rmse):
        current_time = self.get_clock().now().to_msg()

        # Publish the calculated RMSE
        rmse_msg = Float32()
        rmse_msg.data = rmse
        self.rmse_pub.publish(rmse_msg)

        # Log RMSE with timestamp
        self.get_logger().info(f"Timestamp: {current_time.sec}.{current_time.nanosec}, Current RMSE: {rmse}")


    def publish_FASTLIO_RMSE(self, rmse):
        current_time = self.get_clock().now().to_msg()

        # Publish the calculated RMSE
        rmse_msg = Float32()
        rmse_msg.data = rmse
        self.fastlio_rmse_pub.publish(rmse_msg)

        # Log RMSE with timestamp
        self.get_logger().info(f"Timestamp: {current_time.sec}.{current_time.nanosec}, Current FASTLIO RMSE: {rmse}")

    def publish_GLIM_RMSE(self, glim_rmse):
        current_time = self.get_clock().now().to_msg()

        # Publish the calculated RMSE
        glim_rmse_msg = Float32()
        glim_rmse_msg.data = glim_rmse
        self.glim_rmse_pub.publish(glim_rmse_msg)

        # Log RMSE with timestamp
        self.get_logger().info(f"Timestamp: {current_time.sec}.{current_time.nanosec}, Current GLIM RMSE: {glim_rmse}")

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
