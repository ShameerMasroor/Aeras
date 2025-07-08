import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import csv
import os
from datetime import datetime

class YawDiffLogger(Node):
    def __init__(self):
        super().__init__('yaw_diff_logger')
        
        # Create the yaw_statistics directory if it doesn't exist
        os.makedirs('yaw_statistics', exist_ok=True)

        # Create a timestamped CSV file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join('yaw_statistics', f'yaw_diff_log_{timestamp}.csv')

        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Index', 'Time (s)', 'Yaw Difference (rad)'])

        self.counter = 0

        self.subscription = self.create_subscription(
            Float32,
            '/yaw_diffs',
            self.listener_callback,
            10
        )

        self.get_logger().info(f'Logging yaw differences to: {self.csv_path}')

    def listener_callback(self, msg):
        self.counter += 1
        timestamp = self.get_clock().now().nanoseconds / 1e9  # Time in seconds
        self.csv_writer.writerow([self.counter, f"{timestamp:.6f}", msg.data])
        self.csv_file.flush()

        self.get_logger().info(f'[{self.counter}] Time: {timestamp:.2f}s | Yaw diff: {msg.data:.4f} rad')

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YawDiffLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
