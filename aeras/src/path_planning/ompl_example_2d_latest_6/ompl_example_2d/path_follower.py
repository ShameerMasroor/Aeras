#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Pose, Twist, Quaternion, PoseStamped
from nav_msgs.msg import Path
from math import sqrt, atan2

from std_msgs.msg import Bool, Empty  # Import Bool message type


class PoseFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # Declare and get parameters
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('pose_topic', '/simple_drone/gt_pose') #change to slam pose later on
        self.declare_parameter('cmd_vel_topic', '/simple_drone/cmd_vel')
        self.declare_parameter('orientation_topic', '/simple_drone/desired_orientation')
        self.declare_parameter('threshold', 0.5)  #at what distance should the drone consider to have reached the waypoint
        self.declare_parameter('interpolation_factor', 0.5)  # parameter for interpolation
        self.declare_parameter('path_mode_topic', '/simple_drone/posctrl')
        self.declare_parameter('takeoff_topic', '/simple_drone/takeoff')
        self.declare_parameter('find_path_topic', '/find_path')

        self.path_topic = self.get_parameter('path_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.orientation_topic = self.get_parameter('orientation_topic').value
        self.threshold = self.get_parameter('threshold').value
        self.interpolation_factor = self.get_parameter('interpolation_factor').value  # Get interpolation factor
        self.path_mode_topic = self.get_parameter('path_mode_topic').value
        self.takeoff_topic = self.get_parameter('takeoff_topic').value
        self.find_path_topic = self.get_parameter('find_path_topic').value

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.orientation_pub = self.create_publisher(Quaternion, self.orientation_topic, 10)
        self.cmd_quat_pub = self.create_publisher(Quaternion, '/cmd_quat', 10)
        self.takeoff_pub = self.create_publisher(Empty, self.takeoff_topic, 10)
        self.find_path_pub = self.create_publisher(Bool, self.find_path_topic, 10)

        # Publisher for enabling the position controller
        self.path_mode_pub = self.create_publisher(Bool, self.path_mode_topic, 10)

        self.path_sub = self.create_subscription(Path, self.path_topic, self.path_callback, 10)
        self.pose_sub = self.create_subscription(Pose, self.pose_topic, self.pose_callback, 10)

        # Internal state
        self.waypoints = []
        self.rpy = []
        self.quat = []
        self.current_waypoint_index = 0
        self.following_path = False  # Flag to track if a path is being followed

        message = Bool()
        message.data = True
        self.find_path_pub.publish(message)
        self.get_logger().info("Published 'True' to find_path_topic for the first time.")

        # Publish a Bool message to enable position control
        # self.enable_position_control()
        # self.takeoff()   #issue command for takeoff
        self.pose_received = False

        self.get_logger().info("PoseFollower ROS 2 node has been started.")

    def enable_position_control(self):
        # Create and publish the Bool message
        message = Bool()
        message.data = True
        self.path_mode_pub.publish(message)
        self.get_logger().info("Published 'True' to path_mode_topic to enable position control.")


    def takeoff(self):

        message = Empty()
        # message.data = "{}"
        self.takeoff_pub.publish(message)
        self.get_logger().info("Drone takeoff command issued.")


    def path_callback(self, msg: Path):
        """Callback to receive and store waypoints from the planned path."""
        if self.following_path:
            self.get_logger().warn("Cannot accept new path; still following the current path.")
            return

        
        self.waypoints = msg.poses
        self.waypoints = self.interpolate_waypoints(self.waypoints)  # Interpolate waypoints
        self.current_waypoint_index = 0
        self.following_path = bool(self.waypoints)

        if self.following_path:
            self.get_logger().info(f"Received path with {len(self.waypoints)} waypoints.")
        else:
            self.get_logger().warn("Received empty path.")

    def interpolate_waypoints(self, waypoints):
        """Interpolate new waypoints between the ones that are too far apart."""
        new_waypoints = [waypoints[0]]  # Start with the first waypoint
        for i in range(1, len(waypoints)):
            current_waypoint = waypoints[i - 1]
            next_waypoint = waypoints[i]
            distance = self.calculate_distance(current_waypoint.pose, next_waypoint.pose)

            if distance > self.threshold:
                # Interpolate new waypoints
                num_new_points = int(distance // self.threshold)  # Number of points to insert
                for j in range(1, num_new_points + 1):
                    # Interpolate the new waypoint
                    interp_x = current_waypoint.pose.position.x + (next_waypoint.pose.position.x - current_waypoint.pose.position.x) * (j / (num_new_points + 1))
                    interp_y = current_waypoint.pose.position.y + (next_waypoint.pose.position.y - current_waypoint.pose.position.y) * (j / (num_new_points + 1))
                    interp_z = current_waypoint.pose.position.z + (next_waypoint.pose.position.z - current_waypoint.pose.position.z) * (j / (num_new_points + 1))

                    interpolated_pose = Pose()
                    interpolated_pose.position.x = interp_x
                    interpolated_pose.position.y = interp_y
                    interpolated_pose.position.z = interp_z
                    # Use the same orientation as the previous waypoint for simplicity
                    interpolated_pose.orientation = current_waypoint.pose.orientation
                    new_waypoints.append(PoseStamped(pose=interpolated_pose))  # Add the interpolated point

            new_waypoints.append(next_waypoint)  # Add the next waypoint

        return new_waypoints

    def pose_callback(self, msg: Pose):
        """Callback to monitor drone's position and publish waypoints incrementally."""
        if not self.following_path or not self.waypoints:
            return
        
        # if not self.pose_received:
        #     self.pose_received = True
        #     self.get_logger().info("Pose data received from /simple_drone/gt_pose. Issuing takeoff command...")
        #     self.takeoff()

        current_pose = msg
        current_waypoint = self.waypoints[self.current_waypoint_index].pose
        distance_to_waypoint = self.calculate_distance(current_pose, current_waypoint)

        if distance_to_waypoint < self.threshold:
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_index + 1}.")
            self.current_waypoint_index += 1

            if self.current_waypoint_index < len(self.waypoints):
                # Move to the next waypoint
                self.send_twist_and_orientation(current_pose, self.waypoints[self.current_waypoint_index].pose)
                message = Bool()
                message.data = False
                self.find_path_pub.publish(message)
                self.get_logger().info("Published 'False' to find_path_topic.")

            # elif ((len(self.waypoints) - self.current_waypoint_index) == 1):
            #     message = Bool()
            #     message.data = True
            #     self.find_path_pub.publish(message)
            #     self.get_logger().info("Published 'True' to find_path_topic.")


            elif ((self.current_waypoint_index) == len(self.waypoints)//2):
                message = Bool()
                message.data = True
                self.find_path_pub.publish(message)
                self.get_logger().info("Published 'True' to find_path_topic.")



            else:
                self.get_logger().info("All waypoints reached. Stopping the drone.")
                self.waypoints = []  # Reset waypoints
                # message = Bool()
                # message.data = True
                # self.find_path_pub.publish(message)
                # self.get_logger().info("Published 'True' to find_path_topic.")
                self.following_path = False  # Mark path as completed  #change this to false for proper path to path continuation
        else:
            # Keep moving towards the current waypoint
            self.send_twist_and_orientation(current_pose, current_waypoint)

    def calculate_distance(self, current_pose: Pose, target_pose: Pose):
        """Calculate Euclidean distance between current pose and target pose (2D)."""
        return sqrt(
            (current_pose.position.x - target_pose.position.x) ** 2 +
            (current_pose.position.y - target_pose.position.y) ** 2 +
            (current_pose.position.z - target_pose.position.z) ** 2
        )

    def send_twist_and_orientation(self, current_pose: Pose, target_pose: Pose):
        """Calculate and publish Twist commands and desired orientation (quaternion) to move towards the waypoint."""
        cmd = Twist()

        # Calculate the direction and distance
        delta_x = target_pose.position.x - current_pose.position.x
        delta_y = target_pose.position.y - current_pose.position.y

        distance = self.calculate_distance(current_pose, target_pose)

        # Assign the position commands to linear velocity
        cmd.linear.x = target_pose.position.x
        cmd.linear.y = target_pose.position.y
        cmd.linear.z = target_pose.position.z

        # Calculate desired yaw
        desired_yaw = atan2(delta_y, delta_x)
        self.rpy = [0, 0, desired_yaw]
        self.quat = self.euler_to_quaternion(self.rpy)

        quat_cmd = Quaternion()
        quat_cmd.x = self.quat[0]
        quat_cmd.y = self.quat[1]
        quat_cmd.z = self.quat[2]
        quat_cmd.w = self.quat[3]

        self.cmd_quat_pub.publish(quat_cmd)
        self.get_logger().debug(f"Published Quaternion command: {quat_cmd}")

        self.cmd_vel_pub.publish(cmd)
        self.get_logger().debug(f"Published Twist command: {cmd}")

    def euler_to_quaternion(self, rpy):
        (roll, pitch, yaw) = (rpy[0], rpy[1], rpy[2])
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = PoseFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("PoseFollower node shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

