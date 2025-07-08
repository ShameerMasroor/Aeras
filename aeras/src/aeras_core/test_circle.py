import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import tf2_ros
import numpy as np
from geometry_msgs.msg import Point

class HumanDetectionNode(Node):
    def __init__(self):
        super().__init__('human_detection_node')

        self.score_sub = self.create_subscription(Float32, '/human_detection/conf_score', self.score_callback, 10)
        self.image_sub = self.create_subscription(Image, '/human_detection/image', self.image_callback, 10)
        self.marker_pub = self.create_publisher(Marker, '/detection_circle', 10)

        self.bridge = CvBridge()
        self.latest_image = None
        self.conf_threshold = 0.90
        self.detection_triggered = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timer to check for transforms and publish marker
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.publish_once = True

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def score_callback(self, msg):
        if msg.data >= self.conf_threshold and self.latest_image is not None:
            alert_img = self.latest_image.copy()
            cv2.putText(alert_img, f'Person Detected ({msg.data:.2f})',
                        (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            cv2.imshow("Alert", alert_img)
            self.get_logger().warn('Alert')
            cv2.waitKey(1)  # This keeps the OpenCV window responsive

            self.detection_triggered = True  # Flag to trigger marker in timer

    def timer_callback(self):
        # Call waitKey here too so the GUI is refreshed regularly
        cv2.waitKey(1)

        if not self.detection_triggered:
            return

        try:
            transform = self.tf_buffer.lookup_transform('world', 'simple_drone/base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            cx = transform.transform.translation.x
            cy = transform.transform.translation.y
            radius = 5.0

            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'detection_area'
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.1

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
                pt.z = transform.transform.translation.z
                marker.points.append(pt)

            if self.publish_once == True:
                self.marker_pub.publish(marker)
                self.publish_once = False

            self.detection_triggered = False  # Only publish once unless triggered again

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
