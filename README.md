# LI-SLAM and FASTLIO2 for Teleoperated UAVs

Welcome to our Mobile Robotics Project! This repository contains all the necessary resources to build and operate our project. The project leverages **ROS2**, **Gazebo Classic 11**, and **RVIZ2**, along with state of the art SLAM algorithms for mapping and localization in forest environments.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Key Packages](#key-packages)
- [Acknowledgements](#acknowledgements)
- [Getting Started](#getting-started)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Contributing](#contributing)
- [License](#license)

## Overview

This project aims to create a versatile drone platform for research and development purposes pertaining to mapping and localization in forests. The drone is designed to:

- Map a simulated forest environment in Gazebo.
- Perform Lidar odometry for a UAV.

## Features

- **Simulation:** Full integration with Gazebo Classic 11 for realistic environment simulation.
- **Visualization:** Interactive 3D visualization using RVIZ2.
- **Teleoperation:** ROS2 nodes for autonomous flight, including path planning and obstacle avoidance.
- **Modularity:** Easily extendable architecture to add new sensors, controllers, or behaviors.



### Key Packages:

1. **SJTU Drone**: Contains UAV controller and model files. Also contains Gazebo forest world files.
2. **LI-SLAM**: Implements LiDAR Inertial odometry and mapping at the front-end and uses pose-graph as the backend.
3. **FASTLIO 2**: A computationally efficient LiDAR-inertial odometry framework for high-speed, real-time localization and mapping.

### Acknowledgements:
We woukd like to acknowledge the following for the use of their work in this project:
1. Shanghai Jiao Tong University for their SJTU Drone model based on the Parrot AR2 drone (https://github.com/NovoG93/sjtu_drone)
2. Ryuhei Sasaki for his LI-SLAM package (https://github.com/rsasaki0109/li_slam_ros2)
3. HKU-MARS for their FASTLIO Package (https://github.com/hku-mars/FAST_LIO/tree/ROS2)

## Getting Started

### Prerequisites

- [ROS2 (Humble)](https://docs.ros.org/en/humble/index.html)
- [Gazebo Classic 11](http://gazebosim.org/)
- Python 3.8 or later
- C++ (if required for performance-critical nodes)
- Gazebo Model Files available at https://drive.google.com/drive/folders/1m94-q4EyJTxHPq3p_1vGZ0i0HHRN0Vhf?usp=sharing

### Cloning the Repository

```bash
# Clone this repository
$ git clone (https://github.com/ShameerMasroor/LI-SLAM-and-FASTLIO2-for-Teleoperated-UAVs.git

```

## Installation

### 1. Install Dependencies

Use `rosdep` to install dependencies:

```bash
# Install ROS2 dependencies
$ rosdep install --from-paths src --ignore-src -r -y
```

### 2. Build the Workspace

```bash
# Source ROS2 setup
$ source /opt/ros/humble/setup.bash 

# Build the workspace
$ colcon build
```
In case you are unable to build the packages altogether, it is recommended to build the packages one at a time using

```bash
$ cd lidarslam_ros2-develop
$ colcon build --packages-select lidarslam_msgs
$ colcon build --packages-select graph_based_slam
$ colcon build --packages-select scanmatcher
$ colcon build

#Once complete, cd into the sjtu drone directory and run

#colcon build

```



### 3. Launch Simulation

```bash

# Launch the Gazebo simulation
$cd sjtu_drone-ros2

# Source the workspace
$ source install/setup.bash

$ ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py


```

### 4. Launch LI-SLAM Package

```bash

# Launch the Gazebo simulation
$cd lidarslam_ros2-develop

# Source the workspace
$ source install/setup.bash

$ ros2 launch lidarslam lidarslam.launch.py

```


## Usage

### Running the Drone

1. Start the simulation:

   ```bash
   ros2 launch drone_project simulation.launch.py
   ```

2. Start the autonomous navigation node:

   ```bash
   ros2 run drone_project navigation_node
   ```

3. Monitor the simulation in RVIZ2:

   ```bash
   rviz2 -d rviz_config.rviz
   ```

### Example Commands

- Send a mission command:

  ```bash
  ros2 topic pub /mission_command geometry_msgs/msg/PoseStamped "{...}"
  ```

- Monitor telemetry:

  ```bash
  ros2 topic echo /drone/telemetry
  ```



## License

This project is licensed under the MIT License. See the `LICENSE` file for details.
