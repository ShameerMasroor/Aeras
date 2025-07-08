#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Pose, Twist, Quaternion, PoseStamped
from gazebo_msgs.msg import ContactsState
from nav_msgs.msg import Path
from math import sqrt, atan2, pi
from tf_transformations import euler_from_quaternion  #sudo apt update
#sudo apt install ros-humble-tf-transformations

from std_msgs.msg import Bool, Empty, Float32  # Import Bool message type
from time import time
import threading
import tkinter as tk


class PoseFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        #for the collision bumper
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.bumper_subscriber = self.create_subscription(
            ContactsState, '/simple_drone/bumper_states', self.bumper_callback, 10)

        # Declare and get parameters
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('pose_topic', '/lio_pose') #change to slam pose later on
        self.declare_parameter('cmd_vel_topic', '/simple_drone/cmd_vel')
        self.declare_parameter('orientation_topic', '/simple_drone/desired_orientation')
        self.declare_parameter('threshold', 0.5)  #at what distance should the drone consider to have reached the waypoint
        self.declare_parameter('interpolation_factor', 0.1)  # parameter for interpolation
        self.declare_parameter('path_mode_topic', '/simple_drone/posctrl')
        self.declare_parameter('takeoff_topic', '/simple_drone/takeoff')
        self.declare_parameter('find_path_topic', '/find_path')
        self.declare_parameter('heat_angle_topic', '/heat_signature_angle')
        self.declare_parameter('simulation_control_topic', '/simulation_control')
        self.declare_parameter('corrected_path_topic', '/corrected_path')

        #for testing the backtracking functionality
        self.declare_parameter('test_backtrack_topic','/backtrack')

        #for path debugging
        self.interpolated_path_pub = self.create_publisher(Path, '/interpolated_path', 10)


        self.declare_parameter('max_roll', 0.5)  #0.5 radians
        self.declare_parameter('max_pitch', 0.5) #0.5 radians

        self.max_roll = self.get_parameter('max_roll').value
        self.max_pitch = self.get_parameter('max_pitch').value
        

        self.path_topic = self.get_parameter('path_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.orientation_topic = self.get_parameter('orientation_topic').value
        self.threshold = self.get_parameter('threshold').value
        self.interpolation_factor = self.get_parameter('interpolation_factor').value  # Get interpolation factor

        self.backtrack_topic = self.get_parameter('test_backtrack_topic').value

        self.path_mode_topic = self.get_parameter('path_mode_topic').value
        self.takeoff_topic = self.get_parameter('takeoff_topic').value
        self.find_path_topic = self.get_parameter('find_path_topic').value
        self.heat_angle_topic = self.get_parameter('heat_angle_topic').value
        self.simulation_control_topic = self.get_parameter('simulation_control_topic').value
        self.corrected_path_topic = self.get_parameter('corrected_path_topic').value

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.orientation_pub = self.create_publisher(Quaternion, self.orientation_topic, 10)
        self.cmd_quat_pub = self.create_publisher(Quaternion, '/cmd_quat', 10)
        self.takeoff_pub = self.create_publisher(Empty, self.takeoff_topic, 10)
        self.find_path_pub = self.create_publisher(Bool, self.find_path_topic, 10)
        self.simulation_control_pub = self.create_publisher(Bool, self.simulation_control_topic, 10)
        self.corrected_path_sub = self.create_subscription(Path, self.corrected_path_topic, self.corrected_path_callback,10)
        

        # Publisher for enabling the position controller
        self.path_mode_pub = self.create_publisher(Bool, self.path_mode_topic, 10)

        self.path_sub = self.create_subscription(Path, self.path_topic, self.path_callback, 10)
        self.pose_sub = self.create_subscription(Pose, self.pose_topic, self.pose_callback, 10)
        self.backtrack_sub = self.create_subscription(Bool, self.backtrack_topic, self.move_backwards, 10)
        self.heat_angle_sub = self.create_subscription(Float32, self.heat_angle_topic, self.heat_angle_callback, 10)
        self.detection_sub = self.create_subscription(Bool, '/something_detected', self.detection_callback, 10)

        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])


        # self.orientation_timer = self.create_timer(0.1, self.publish_heat_yaw)

        # Internal state
        self.waypoints = []
        self.rpy = []
        self.quat = []
        self.current_waypoint_index = 0
        self.following_path = False  # Flag to track if a path is being followed
        self.is_flying = False
        self.is_at_altitude = False
        self.reversing = False
        self.heat_controlled_yaw = False

        #this is to prevent the backtrack function from being triggered repeatedly while the collision is occuring
        self.backup_once = True

        message = Bool()
        message.data = True
        self.find_path_pub.publish(message)
        self.get_logger().info("Published 'True' to find_path_topic for the first time.")

        # Publish a Bool message to enable position control
        # self.enable_position_control()
        # self.takeoff()   #issue command for takeoff
        self.path_received = False

        self.get_logger().info("PoseFollower ROS 2 node has been started.")
        #this timer is to repeatly sent True to OMPL until a path is received. This is only for initialization
        self.timer = self.create_timer(2.0, self.check_and_request_path)  # Check every 1 second


        self.declare_parameter('path_resume_delay', 5.0)  # Default delay of 5 seconds
        self.path_resume_delay = self.get_parameter('path_resume_delay').value

        # ... (other initializations)

        self.path_following_pending = False  # Flag for pending path following
        self.path_start_timer = None  # Timer for delay
        self.path_paused = False  # Flag for paused path following
        self.path_resume_timer = None  # Timer for resume delay
        self.heat_signature_path = False
        self.heat_angle = 0.0
        self.desired_yaw = 0.0
        self.corrected_waypoints = []

        self.sim_start_time = self.get_clock().now()
 

        # Launch Tkinter window in separate thread
        threading.Thread(target=self.init_tkinter_window, daemon=True).start()
        self.timer_sim_time = self.create_timer(0.1, self.update_sim_time)

    def init_tkinter_window(self):
        self.tk_root = tk.Tk()
        self.tk_root.title("Simulation Time")

        self.sim_elapsed_time = tk.StringVar()
        self.sim_elapsed_time.set("Sim Time: 0.00s")

        label = tk.Label(self.tk_root, textvariable=self.sim_elapsed_time, font=("Helvetica", 16))
        label.pack(padx=20, pady=20)

        self.tk_root.mainloop()

    def update_sim_time(self):
        current_time = self.get_clock().now()
        elapsed = (self.get_clock().now() - self.sim_start_time).nanoseconds / 1e9
        self.sim_elapsed_time.set(f"Sim Time: {elapsed:.2f}s")

        if (elapsed >=300):
            message = Bool()
            message.data = True  # Or False, based on your logic
            self.simulation_control_pub.publish(message)




    def check_and_request_path(self):
        """Checks if a path has been received and publishes 'True' to find_path if not."""
        if not self.path_received:
            message = Bool()
            message.data = True
            self.find_path_pub.publish(message)
            self.get_logger().info("Published 'True' to find_path_topic to request a path.")

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
        self.is_flying = True
        self.get_logger().info("Drone takeoff command issued.")


    def path_callback(self, msg: Path):
        """Callback to receive and store waypoints from the planned path."""
        self.enable_position_control()
        self.takeoff()
        if self.following_path and not self.heat_signature_path:
            self.get_logger().warn("Cannot accept new path; still following the current path.")
            return

        if not msg.poses:  # Check if the path is empty
            self.get_logger().warn("Received empty path.")
            message = Bool()
            message.data = True
            self.find_path_pub.publish(message)
            self.get_logger().info("Published 'True' to find_path_topic due to empty path.")
            return
        
        # if len(msg.poses) == 2:  # Reject paths with exactly 2 waypoints
        #     self.get_logger().warn("Received a path with exactly 2 waypoints, ignoring.")
        #     message = Bool()
        #     message.data = True
        #     self.find_path_pub.publish(message)
        #     self.get_logger().info("Published 'True' to find_path_topic due to invalid path length.")
        #     return



        self.waypoints = msg.poses
        self.waypoints = self.interpolate_waypoints(self.waypoints)  # Interpolate waypoints
        self.current_waypoint_index = 0
        self.following_path = bool(self.waypoints)

        if self.following_path:
            self.get_logger().info(f"Received path with {len(self.waypoints)} waypoints.")
            if self.timer is not None: #check if the timer exists before cancelling.
                self.timer.cancel() #stop the timer, a path has been received.
                self.timer = None #set timer to none so it cant be accidently cancelled again.
        else:
            self.get_logger().warn("Received empty path.")

    def corrected_path_callback(self, msg: Path):
            """Callback to receive and store waypoints from the planned path."""

            self.corrected_waypoints = msg.poses
            # self.waypoints = self.interpolate_waypoints(self.waypoints)  # Interpolate waypoints
            # self.current_waypoint_index = 0
            # self.following_path = bool(self.corrected_waypoints)

    def interpolate_waypoints(self, waypoints):
        """Interpolate new waypoints between the ones that are too far apart."""
        new_waypoints = [waypoints[0]]  # Start with the first waypoint
        for i in range(1, len(waypoints)):
            current_waypoint = waypoints[i - 1]
            next_waypoint = waypoints[i]
            distance = self.calculate_distance(current_waypoint.pose, next_waypoint.pose)

            if distance > self.threshold:
                # Interpolate new waypoints
                num_new_points = int(distance // self.interpolation_factor)  # Number of points to insert
                for j in range(1, num_new_points + 1):
                    # Interpolate the new waypoint
                    interp_x = current_waypoint.pose.position.x + (next_waypoint.pose.position.x - current_waypoint.pose.position.x) * (j / (num_new_points + 1))
                    interp_y = current_waypoint.pose.position.y + (next_waypoint.pose.position.y - current_waypoint.pose.position.y) * (j / (num_new_points + 1))
                    interp_z = current_waypoint.pose.position.z + (next_waypoint.pose.position.z - current_waypoint.pose.position.z) * (j / (num_new_points + 1))

                    interpolated_pose = PoseStamped()
                    interpolated_pose.header.stamp = self.get_clock().now().to_msg()
                    interpolated_pose.header.frame_id = "map"
                    interpolated_pose.pose.position.x = interp_x
                    interpolated_pose.pose.position.y = interp_y
                    interpolated_pose.pose.position.z = interp_z
                    interpolated_pose.pose.orientation = current_waypoint.pose.orientation  # Keep same orientation
                    
                    new_waypoints.append(interpolated_pose)  # Add interpolated waypoint

            new_waypoints.append(next_waypoint)  # Add the next waypoint

        # Publish the interpolated path
        interpolated_path = Path()
        interpolated_path.header.stamp = self.get_clock().now().to_msg()
        interpolated_path.header.frame_id = "map"
        interpolated_path.poses = new_waypoints

        self.interpolated_path_pub.publish(interpolated_path)
        self.get_logger().info(f"Published interpolated path with {len(new_waypoints)} waypoints.")

        return new_waypoints


    def pose_callback(self, msg: Pose):
        """Callback to monitor drone's position and publish waypoints incrementally."""
        if not self.following_path or not self.waypoints:
            return

        if self.current_waypoint_index >= len(self.corrected_waypoints):
            self.get_logger().warn("Current waypoint index out of range. Ignoring callback.")
            return

        current_pose = msg
        current_waypoint = self.corrected_waypoints[self.current_waypoint_index].pose

        if current_pose.position.z > 0.1:
            self.is_at_altitude = True
        else:
            self.is_at_altitude = False


        # Convert quaternion to Euler angles
        roll, pitch, yaw = euler_from_quaternion([
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w
        ])

        # Check roll and pitch limits
        if abs(roll) > self.max_roll or abs(pitch) > self.max_pitch:
            if not self.path_paused: #only pause if not already paused.
                self.get_logger().warn(f"Roll ({roll:.2f}) or pitch ({pitch:.2f}) exceeded limits. Pausing path following.")
                # self.stop_drone()
                self.path_paused = True
                self.path_resume_timer = self.create_timer(self.path_resume_delay, self.resume_path_following)
            return

        if self.path_paused:
            return #do not continue following the path, until the path is resumed.

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
                # self.get_logger().info("Published 'False' to find_path_topic.")
            else:
                if self.reversing:
                    self.reversing = False
                    message = Bool()
                    message.data = True
                    self.find_path_pub.publish(message)
                    self.get_logger().info("Reversal completed. Publishing 'True' to find_path_topic.")
                    self.waypoints = [] # reset waypoints
                    self.corrected_waypoints = []
                    self.following_path = False
                    self.backup_once = True
                    self.heat_signature_path = False
                else:
                    self.get_logger().info("All waypoints reached. Stopping the drone.")
                    self.waypoints = []  # Reset waypoints
                    self.corrected_waypoints = []
                    message = Bool()
                    message.data = True
                    self.find_path_pub.publish(message)
                    self.get_logger().info("Published 'True' to find_path_topic.")
                    self.following_path = False
                    self.heat_signature_path = False
        else:
            # Keep moving towards the current waypoint
            self.send_twist_and_orientation(current_pose, current_waypoint)

    def calculate_distance(self, current_pose: Pose, target_pose: Pose):
        """Calculate Euclidean distance between current pose and target pose (2D)."""

        #the following condition is to ensure the collision bumper doesnt retrigger path generation when the drone is resting on the ground.
        # self.is_at_altitude = False
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
        # self.heat_controlled_yaw = False   #comment this to allow heat based turning
        if (self.reversing == False): 

            if not self.heat_controlled_yaw:
                self.desired_yaw = atan2(delta_y, delta_x)
            else:
                # Convert relative heat angle to absolute yaw
                _, _, current_yaw = euler_from_quaternion([
                    current_pose.orientation.x,
                    current_pose.orientation.y,
                    current_pose.orientation.z,
                    current_pose.orientation.w
                ])
                
                relative_yaw_rad = self.heat_angle * pi / 180
                self.desired_yaw = current_yaw + relative_yaw_rad


            self.rpy = [0, 0, self.desired_yaw]
            self.quat = self.euler_to_quaternion(self.rpy)



            # self.rpy = [0, 0, self.desired_yaw]
            # self.quat = self.euler_to_quaternion(self.rpy)

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
    
    def bumper_callback(self, msg):
        if (self.is_at_altitude and self.backup_once):
            # self.get_logger().info('In bumper function')
            if msg.states:  # Check if collision
                self.get_logger().warn('Collision detected. Initiating reversal.')
                self.move_backwards()
                self.backup_once = False
        else:
            self.get_logger().info("Drone below collision altitude")



#to trigger this function manually, we need to do: ros2 topic pub --once /backtrack std_msgs/Bool "{data: true}"

    def move_backwards(self, msg=None):
        if not self.waypoints or self.current_waypoint_index <= 1 or not self.is_flying:
            self.get_logger().warn("No waypoints to move back to, at the start, or not flying. Ignoring move_backwards call.")
            return

        self.reversing = True
        #I have set it to reverse 10 waypoints back and then replan the path
        self.corrected_waypoints = self.waypoints[self.current_waypoint_index - 1: self.current_waypoint_index - 25: -1]
        self.current_waypoint_index = 0
        self.get_logger().info("Reversing waypoints for return path.")


    def resume_path_following(self):
        """Timer callback to resume path following after delay."""
        self.path_paused = False
        self.path_resume_timer.cancel()
        self.path_resume_timer = None
        self.get_logger().info("Resuming path following.")
    
    def detection_callback(self, msg):

        if (msg.data):
            self.heat_controlled_yaw = True
            # self.get_logger().info("Heat detected.")
        
            if self.heat_signature_path == False:
                self.heat_signature_path = True
                
                message = Bool()
                message.data = True
                self.find_path_pub.publish(message)
                self.find_path_pub.publish(message)
                self.find_path_pub.publish(message)
                self.get_logger().info("Published 'True' to find_path_topic as heat detected.")

        else:
            self.heat_controlled_yaw = False

    def heat_angle_callback(self, msg):
        self.heat_angle = msg.data

    # def publish_heat_yaw(self):
    #     pass
        # if not self.heat_controlled_yaw:
        #     return

        # desired_yaw =  self.heat_angle * pi/180
        # # desired_yaw = self.heat_angle * (pi / 180)
        # self.rpy = [0, 0, desired_yaw]
        # self.quat = self.euler_to_quaternion(self.rpy)

        # quat_cmd = Quaternion()
        # quat_cmd.x = self.quat[0]
        # quat_cmd.y = self.quat[1]
        # quat_cmd.z = self.quat[2]
        # quat_cmd.w = self.quat[3]

        # self.cmd_quat_pub.publish(quat_cmd)
        # self.get_logger().debug("Publishing heat-controlled yaw.")



            

    
    

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
