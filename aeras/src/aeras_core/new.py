# import numpy as np
# import matplotlib.pyplot as plt
# from scipy.ndimage import gaussian_filter

# # Create empty occupancy grid
# grid_size = 1000
# occupancy_grid = np.zeros((grid_size, grid_size))

# # Define high-probability center points
# hotspots = [(200, 300), (700, 800), (500, 400)]

# # Add high values at hotspot centers
# for x, y in hotspots:
#     occupancy_grid[y, x] = 1.0  # center gets max value

# # Apply Gaussian smoothing
# smoothed_grid = gaussian_filter(occupancy_grid, sigma=70)

# # Optionally normalize between 0 and 1
# smoothed_grid = smoothed_grid / smoothed_grid.max()

# print("AAAAAAAAABBBBBBCCCCCCCC")
# # Visualize it using matplotlib
# plt.imshow(smoothed_grid, cmap='plasma', origin='lower')
# plt.colorbar(label='Probability')
# plt.title("Smoothed Occupancy Grid")
# plt.show()

print("STARTING SCRIPT")  # Add this at the top of new.py
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

class ProbabilityMapPublisher(Node):
    def __init__(self):
        super().__init__('probability_map_publisher')
        self.publisher = self.create_publisher(OccupancyGrid, 'probability_map', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, 'hotspot_centers', 10)
        self.timer = self.create_timer(1.0, self.publish_data)
        self.get_logger().info("Probability Map Publisher Node Started")

        ##self.hotspots = [(200, 300), (700, 800), (500, 400)]  # High-probability centers

        self.hotspots = [(45, 45)]  # High-probability centers
        # index = 45 * 50 + 45;
        # index = 2250 + 45;
        # index = 2295;



        
        

    def generate_probability_map(self):
        grid_size = 50
        occupancy_grid = np.zeros((grid_size, grid_size))

        # Add high values at hotspot centers
        for x, y in self.hotspots:
            occupancy_grid[y, x] = 1.0  # center gets max value

        # Apply Gaussian smoothing
        smoothed_grid = gaussian_filter(occupancy_grid, sigma=5)
        smoothed_grid = smoothed_grid / smoothed_grid.max()  # Normalize to 0-1

        return smoothed_grid
    

    def display_map(self, probability_map):
        plt.ion()  # Turn on interactive mode
        plt.clf()  # Clear the previous plot

        plt.imshow(probability_map, cmap='hot', origin='lower')
        plt.colorbar(label='Probability')
        plt.title("Probability Map")

        plt.draw()  # Draw the updated plot
        plt.pause(0.1)  # Pause briefly to allow updates



    def publish_data(self):
	    probability_map = self.generate_probability_map()
	    self.publish_map()
	    self.publish_hotspots()
	    self.display_map(probability_map)  # Display the map


    def publish_map(self):
        probability_map = self.generate_probability_map()
        
        # Convert to 0-100 scale for OccupancyGrid (-1 is unknown)
        probability_map = (probability_map * 100).astype(np.int8)

        # Create OccupancyGrid message
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        msg.info.resolution = 1.0  # Each cell = 1x1 meters
        msg.info.width = probability_map.shape[1]
        msg.info.height = probability_map.shape[0]
        msg.info.origin.position.x = -25.0  # Centering the origin to the middle of the map
        msg.info.origin.position.y = -25.0
        msg.info.origin.position.z = 0.0  # Typically remains 0.0 for 2D maps


        msg.data = probability_map.flatten().tolist()

        self.publisher.publish(msg)
        self.get_logger().info("Published Probability Map")

    def publish_hotspots(self):
        marker_array = MarkerArray()
        
        for i, (x, y) in enumerate(self.hotspots):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "hotspot_markers"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = float(x)  # Convert to float
            marker.pose.position.y = float(y)  # Convert to float
            marker.pose.position.z = 0.0
            
            marker.scale.x = 5.0
            marker.scale.y = 5.0
            marker.scale.z = 5.0
            
            marker.color.a = 1.0  # Alpha (opacity)
            marker.color.r = 1.0  # Red
            marker.color.g = 0.0
            marker.color.b = 0.0

            marker_array.markers.append(marker)
        
        self.marker_publisher.publish(marker_array)
        self.get_logger().info("Published Hotspot Centers")

def main(args=None):
    rclpy.init(args=args)
    node = ProbabilityMapPublisher()
    node.get_logger().info("Node has started spinning")
    rclpy.spin(node)  # Keeps the node running
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

