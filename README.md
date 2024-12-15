# LI-SLAM and FASTLIO2 for Teleoperated UAVs

Welcome to our Mobile Robotics Project! This repository contains all the necessary resources to build and operate our project. The project leverages **ROS2**, **Gazebo Classic 11**, and **RVIZ2**, along with state of the art SLAM algorithms for mapping and localization in forest environments.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Architecture](#system-architecture)
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
- **Autonomy:** ROS2 nodes for autonomous flight, including path planning and obstacle avoidance.
- **Modularity:** Easily extendable architecture to add new sensors, controllers, or behaviors.
- **Telemetry:** Real-time feedback on position, velocity, and sensor data.


### Key Packages:

1. **SJTU Drone**: Handles mission-specific logic, such as waypoint navigation.
2. **LI-SLAM**: Implements algorithms for trajectory generation and obstacle avoidance.
3. **FASTLIO 2**: Converts planned paths into low-level motor commands.

## Getting Started

### Prerequisites

- [ROS2 (Humble)](https://docs.ros.org/en/humble/index.html)
- [Gazebo Classic 11](http://gazebosim.org/)
- Python 3.8 or later
- C++ (if required for performance-critical nodes)

### Cloning the Repository

```bash
# Clone this repository
$ git clone https://github.com/yourusername/drone-project.git
$ cd drone-project
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

### 3. Launch Simulation

```bash
# Source the workspace
$ source install/setup.bash

# Launch the Gazebo simulation
$ ros2 launch drone_project gazebo.launch.py
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

## Project Structure

```
.
├── src
│   ├── drone_project
│   │   ├── launch
│   │   ├── scripts
│   │   ├── src
│   │   ├── include
│   │   └── CMakeLists.txt
├── config
├── rviz
├── worlds
└── README.md
```

- **`src/`**: Contains ROS2 nodes.
- **`config/`**: Configuration files for parameters.
- **`rviz/`**: RVIZ2 configuration files.
- **`worlds/`**: Custom Gazebo world files.

## Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository.
2. Create a feature branch:

   ```bash
   git checkout -b feature/your-feature
   ```

3. Commit your changes:

   ```bash
   git commit -m "Add your message here"
   ```

4. Push to your fork:

   ```bash
   git push origin feature/your-feature
   ```

5. Submit a pull request.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.
