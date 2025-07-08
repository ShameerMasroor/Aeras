import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
from ultralytics import YOLO
from supervision import Detections
import numpy as np
from std_msgs.msg import Float32, Bool
from time import time
import threading
import tkinter as tk

class HumanDetectionNode(Node):
    def __init__(self):
        super().__init__('human_detection_node')
        
        self.subscription = self.create_subscription(
            Image, '/simple_drone/front/image_raw', self.image_callback, 10)
        
        self.image_publisher = self.create_publisher(Image, '/human_detection/image', 10)
        self.score_publisher = self.create_publisher(Float32, '/human_detection/conf_score', 10)
        self.angle_publisher = self.create_publisher(Float32, '/heat_signature_angle', 10)
        self.detection_pub = self.create_publisher(Bool, 'something_detected', 10)

        self.bridge = CvBridge()
        self.model = YOLO("model.pt")
        self.get_logger().info("Human Detection Node Initialized")
        self.detection_confidence_threshold = 0.5
        self.detection_hold_time = 1.0  # seconds
        self.above_threshold_start_time = None
        self.last_detection_state = False
        self.detection_drop_time = 5.0
        self.below_threshold_start_time = None




    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            red_channel = cv_image[:, :, 2]
            red_image = cv.merge([red_channel, red_channel, red_channel])
            
            model_output = self.model(red_image, conf=0.6, verbose=False)
            detections = Detections.from_ultralytics(model_output[0])
            processed_image, best_angle, best_score = self.draw_bounding_box(red_image, detections)

            image_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding="bgr8")
            self.image_publisher.publish(image_msg)

            score_msg = Float32()
            score_msg.data = float(best_score)
            self.score_publisher.publish(score_msg)

            angle_msg = Float32()
            angle_msg.data = float(best_angle) if best_score > self.detection_confidence_threshold else 0.0
            self.angle_publisher.publish(angle_msg)

            current_time = time()
            detected = self.last_detection_state
            if best_score > self.detection_confidence_threshold:
                self.above_threshold_start_time = self.above_threshold_start_time or current_time
                self.below_threshold_start_time = None

                if current_time - self.above_threshold_start_time >= self.detection_hold_time:
                    detected = True
            else:
                self.above_threshold_start_time = None

                if self.last_detection_state:
                    self.below_threshold_start_time = self.below_threshold_start_time or current_time
                    if current_time - self.below_threshold_start_time >= self.detection_drop_time:
                        detected = False
                else:
                    self.below_threshold_start_time = None
                    detected = False

            if detected != self.last_detection_state:
                detection_msg = Bool()
                detection_msg.data = detected
                self.detection_pub.publish(detection_msg)
                self.last_detection_state = detected

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def draw_bounding_box(self, image, detections):
        best_score = 0.0
        best_angle = 0.0
        height, width, _ = image.shape
        center_x = width // 2
        horizontal_FOV = 120

        for bbox, score, class_name in zip(detections.xyxy, detections.confidence, detections.data["class_name"]):
            x1, y1, x2, y2 = map(int, bbox)
            label = f"{class_name}: {score:.2f}"
            cv.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv.putText(image, label, (x1, y1 - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            centroid_x = (x1 + x2) // 2
            deviation = centroid_x - center_x
            heat_signature_angle = -(deviation / width) * horizontal_FOV

            if score > best_score:
                best_score = float(score)
                best_angle = float(heat_signature_angle)

        return image, best_angle, best_score


def main(args=None):
    rclpy.init(args=args)
    node = HumanDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

