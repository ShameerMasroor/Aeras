from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    package_dir = get_package_share_directory('octomap_server2')
    
    # Path to the map file inside the package
    map_file_path = os.path.join(package_dir, 'maps', 'map2.ot')  # Replace 'example_map.bt' with your map file name
    
    return LaunchDescription([
        Node(
            package='octomap_server2',
            executable='load_octomap_node',
            name='load_octomap',
            output='screen',
            parameters=[
                {'map_file_path': map_file_path}  # Pass the map file path as a parameter
            ]
        )
    ])
