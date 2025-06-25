import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from skimage.measure import regionprops, label

class RedObjectDetector(Node):
    def __init__(self):
        super().__init__('red_object_detector')
        self.image_pub = self.create_publisher(Image, 'processed_image', 10)
        self.yaw_pub = self.create_publisher(Float32, 'heat_signature_angle', 10)
        self.detection_pub = self.create_publisher(Bool, 'something_detected', 10)
        self.image_sub = self.create_subscription(Image, '/simple_drone/front/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.get_logger().info("Red Object Detector Node Started")
    
    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            red, _, _ = cv2.split(img)
            img_binary = red > 200
            
            # Label connected components
            labeled_img = label(img_binary)
            properties = regionprops(labeled_img)
            filtered_regions = [region for region in properties if region.area > 20]
            
            # Compute image center
            height, width = img_binary.shape
            center = np.array([height / 2, width / 2])
            yaw_change = 0.0
            detection_msg = Bool()
            
            if filtered_regions:
                centroid = np.array(filtered_regions[0].centroid)
                deviation = centroid[1] - center[1]
                horizontal_FOV = 120  # Field of View in degrees
                yaw_change = (deviation / width) * horizontal_FOV
                
                # Draw centroid and center
                cv2.circle(img, (int(centroid[1]), int(centroid[0])), 5, (0, 255, 0), -1)
                cv2.circle(img, (int(center[1]), int(center[0])), 5, (255, 0, 0), -1)
                detection_msg.data = True
            else:
                detection_msg.data = False
            
            # Publish processed image
            img_msg = self.bridge.cv2_to_imgmsg(cv2.cvtColor(img, cv2.COLOR_RGB2BGR), encoding='bgr8')
            self.image_pub.publish(img_msg)
            
            # Publish yaw deviation
            yaw_msg = Float32()
            yaw_msg.data = yaw_change
            self.yaw_pub.publish(yaw_msg)
            self.detection_pub.publish(detection_msg)
            
            self.get_logger().info(f"Yaw Change (Degrees): {yaw_change}")
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")
            

def main(args=None):
    rclpy.init(args=args)
    node = RedObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()