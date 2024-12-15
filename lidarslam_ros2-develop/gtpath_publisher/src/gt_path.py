#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path

class PoseToPathNode(Node):
    def __init__(self):
        super().__init__('pose_to_path')

        # Declare and get parameters
        self.declare_parameter('pose_topic', '/simple_drone/gt_pose')
        self.declare_parameter('path_topic', '/gt_path')
        self.declare_parameter('frame_id', 'map')

        pose_topic = self.get_parameter('pose_topic').value
        path_topic = self.get_parameter('path_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        # Initialize the Path message
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id

        # Create a publisher for the Path message
        self.path_pub = self.create_publisher(Path, path_topic, 10)

        # Create a subscription to the Pose message
        self.pose_sub = self.create_subscription(Pose, pose_topic, self.pose_callback, 10)

        self.get_logger().info(f"PoseToPathNode initialized with pose_topic='{pose_topic}' and path_topic='{path_topic}'.")

    def pose_callback(self, pose_msg):
        # Create a PoseStamped from the incoming Pose message
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = self.frame_id
        pose_stamped.pose = pose_msg

        # Append the PoseStamped to the Path message
        self.path_msg.poses.append(pose_stamped)

        # Update the Path header
        self.path_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the Path message
        self.path_pub.publish(self.path_msg)

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