print("STARTING SCRIPT")

import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose

class ProbabilityMapPublisher(Node):
    def __init__(self):
        super().__init__('probability_map_publisher')
        self.publisher = self.create_publisher(OccupancyGrid, 'probability_map', 10)
        self.timer = self.create_timer(1.0, self.publish_data)
        self.get_logger().info("Probability Map Publisher Node Started")

        self.map_path = "merged_prob_map_abm.npy"  # Path to your .npy file
        self.origin = (-25.0, -25.0)
        self.resolution = 1.0

    def load_probability_map(self):
        try:
            array = np.load(self.map_path)
            array = array / np.max(array)  # Normalize to 0–1
            return array
        except Exception as e:
            self.get_logger().error(f"Failed to load probability map: {e}")
            return np.zeros((50, 50))  # fallback

    def publish_data(self):
        probability_map = self.load_probability_map()
        self.publish_map(probability_map)
        self.display_map(probability_map)

    def publish_map(self, array):
        scaled = (array * 100).astype(np.int8)

        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        msg.info.resolution = self.resolution
        msg.info.width = array.shape[1]
        msg.info.height = array.shape[0]

        msg.info.origin = Pose()
        msg.info.origin.position.x = self.origin[0]
        msg.info.origin.position.y = self.origin[1]
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = scaled.flatten().tolist()
        self.publisher.publish(msg)
        self.get_logger().info("Published loaded .npy probability map")

    def display_map(self, probability_map):
        plt.ion()
        plt.clf()
        plt.imshow(probability_map, cmap='hot', origin='lower')
        plt.colorbar(label='Probability')
        plt.title("Loaded Probability Map from .npy")
        plt.draw()
        plt.pause(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = ProbabilityMapPublisher()
    node.get_logger().info("Node has started spinning")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
