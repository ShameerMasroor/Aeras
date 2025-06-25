import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import tf2_ros
import numpy as np
from geometry_msgs.msg import Point
import tkinter as tk
from tkinter import messagebox
import threading
import csv
import os
from datetime import datetime
from gazebo_msgs.msg import ContactsState


class HumanDetectionNode(Node):
    def __init__(self):
        super().__init__('human_detection_node')

        self.score_sub = self.create_subscription(Float32, '/human_detection/conf_score', self.score_callback, 10)
        self.image_sub = self.create_subscription(Image, '/human_detection/image', self.image_callback, 10)
        
        self.marker_pub = self.create_publisher(Marker, '/detection_circle', 10)
        self.pause_mission_pub = self.create_publisher(Bool, '/pause_mission', 10)
        self.simulation_control_pub = self.create_publisher(Bool, '/simulation_control', 10)

        self.bridge = CvBridge()
        self.latest_image = None
        self.conf_threshold = 0.90
        self.should_draw_circle = False
        self.confidence_reached = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sim_start_time = self.get_clock().now()
        self.detection_time = None
        self.elapsed_time = None

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.save_once = True
        self.human_detected = 0

        self.log_folder = 'human_detection_times'
        os.makedirs(self.log_folder, exist_ok=True)

        timestamp_str = datetime.now().strftime("%Y%m%d_%H-%M")
        self.filename = os.path.join(self.log_folder, f'detection_log_{timestamp_str}.csv')

        with open(self.filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Sim Time (sec)', 'Sim Time (nsec)', 'Human Detected'])


    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def score_callback(self, msg):
        current_time = self.get_clock().now()
        sim_time_sec = current_time.seconds_nanoseconds()[0]
        sim_time_nsec = current_time.seconds_nanoseconds()[1]
        

        if not self.confidence_reached and msg.data >= self.conf_threshold:
            self.detection_time = current_time
            self.elapsed_time = (self.detection_time - self.sim_start_time).nanoseconds / 1e9
            self.confidence_reached = True
            self.get_logger().info(f"Confidence threshold reached at {self.elapsed_time:.2f} seconds")

        if msg.data >= self.conf_threshold:
            self.human_detected = 1

        # Log the current simulation time and detection state
        with open(self.filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([sim_time_sec, sim_time_nsec, self.human_detected])

        if self.human_detected and self.latest_image is not None:
            alert_img = self.latest_image.copy()
            cv2.putText(alert_img, f'Person Detected ({msg.data:.2f})',
                        (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            cv2.imshow("Alert", alert_img)
            self.get_logger().warn('Person Detected!')
            cv2.waitKey(1)

            self.pause_mission_pub.publish(Bool(data=True))
            threading.Thread(target=self.ask_user_continue).start()

    def publish_pause(self, should_pause):
        self.pause_mission_pub.publish(Bool(data=should_pause))

    def ask_user_continue(self):
        root = tk.Tk()
        root.withdraw()

        result = messagebox.askyesno("Mission Alert", "Object detected!\nContinue mission?")
        self.simulation_control_pub.publish(Bool(data=True))

        if result:
            self.publish_pause(False)
        else:
            self.publish_pause(True)
            self.should_draw_circle = True

        root.destroy()

    def timer_callback(self):
        cv2.waitKey(1)

        if not self.should_draw_circle:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'world', 'simple_drone/base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))

            cx = transform.transform.translation.x
            cy = transform.transform.translation.y
            cz = transform.transform.translation.z
            radius = 5.0

            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'detection_area'
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.2
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            num_points = 36
            for i in range(num_points + 1):
                angle = 2 * np.pi * i / num_points
                pt = Point()
                pt.x = cx + radius * np.cos(angle)
                pt.y = cy + radius * np.sin(angle)
                pt.z = cz
                marker.points.append(pt)

            self.marker_pub.publish(marker)
            self.should_draw_circle = False

        except Exception as e:
            self.get_logger().warn(f'Could not get transform: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = HumanDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
