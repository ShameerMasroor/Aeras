# **LI-SLAM and FASTLIO2 for Teleoperated UAVs**  

Welcome to our **Mobile Robotics Project!** This repository contains all the necessary resources to simulate, operate, and extend our teleoperated UAV system for mapping and localization in forest environments. The project leverages **ROS2**, **Gazebo Classic 11**, **RVIZ2**, and cutting-edge SLAM algorithms to deliver state-of-the-art performance.

---

## **Table of Contents**  
- [Overview](#overview)  
- [Features](#features)  
- [Key Packages](#key-packages)  
- [Acknowledgements](#acknowledgements)  
- [Getting Started](#getting-started)  
- [Installation](#installation)  
- [Usage](#usage)
- [People behind the project] (#people)


---

## **Overview**  

This project focuses on creating a versatile UAV platform for mapping and localization in GPS-denied environments, particularly forests. The drone is designed to:  

- Map simulated forest environments in **Gazebo**.  
- Perform **LiDAR-Inertial odometry** for precise localization.  

---

## **Features**  

- **Simulation:** Full integration with **Gazebo Classic 11** for realistic forest environments.  
- **Visualization:** 3D mapping and localization visualization using **RVIZ2**.  
- **Teleoperation:** ROS2 nodes for autonomous flight, obstacle avoidance, and manual control.  
- **Scalable Architecture:** Modular design for adding new sensors, controllers, or behaviors.  

---

## **Key Packages**  

1. **SJTU Drone:** Contains UAV controller, simulation models, and forest world files.  
2. **LI-SLAM:** Implements LiDAR-Inertial odometry and mapping with a pose-graph backend.  
3. **FASTLIO2:** High-speed, real-time LiDAR-Inertial odometry for efficient localization and mapping.  

---

## **Acknowledgements**  

We would like to acknowledge the following for their contributions:  

1. **Shanghai Jiao Tong University** for the **SJTU Drone** model, based on the Parrot AR2 drone: [Repository](https://github.com/NovoG93/sjtu_drone)  
2. **Ryuhei Sasaki** for the **LI-SLAM** package: [Repository](https://github.com/rsasaki0109/li_slam_ros2)  
3. **HKU-MARS** for the **FASTLIO2** package: [Repository](https://github.com/hku-mars/FAST_LIO/tree/ROS2)  

---

## **Getting Started**  

### **Prerequisites**  

- [ROS2 Humble](https://docs.ros.org/en/humble/index.html)  
- [Gazebo Classic 11](http://gazebosim.org/)  
- Python 3.8 or later  
- C++ (for performance-critical nodes)  
- [Gazebo Model Files](https://drive.google.com/drive/folders/1m94-q4EyJTxHPq3p_1vGZ0i0HHRN0Vhf?usp=sharing)  

---

### **Cloning the Repository**  

```bash  
# Clone the repository  
git clone https://github.com/ShameerMasroor/LI-SLAM-and-FASTLIO2-for-Teleoperated-UAVs.git  
```  

---

## **Installation**  

### **1. Install Dependencies**  

Use `rosdep` to install dependencies:  

```bash  
# Install ROS2 dependencies  
rosdep install --from-paths src --ignore-src -r -y  
```  

### **2. Download Gazebo Models**  

1. Visit [this Google Drive link](https://drive.google.com/drive/folders/1m94-q4EyJTxHPq3p_1vGZ0i0HHRN0Vhf?usp=sharing).  
2. Download the model files and place them in the following Gazebo directories:  

   ```bash  
   /usr/share/gazebo-11/models  
   /usr/share/gazebo-11/materials  
   ```  

> **Warning:** The Gazebo simulation will fail to load if the models are not placed correctly!  

### **3. Build the Workspace**  

```bash  
# Source ROS2 setup  
source /opt/ros/humble/setup.bash  

# Build the workspace  
colcon build  
```  

If errors occur during the build, you can build individual packages manually:  

```bash  
# Build specific packages
cd lidarslam_ros2-develop
colcon build --packages-select lidarslam_msgs  
colcon build --packages-select graph_based_slam  
colcon build --packages-select scanmatcher  

# Build SJTU Drone package  
cd sjtu_drone  
colcon build  
```  

---

## **Usage**  

### **1. Launch the Simulation**  

```bash  
# Navigate to the SJTU Drone package  
cd sjtu_drone-ros2  

# Source the workspace  
source install/setup.bash  

# Launch the Gazebo simulation  
ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py  
```  

### **2. Launch LI-SLAM**  

```bash  
# Navigate to the LI-SLAM package  
cd lidarslam_ros2-develop  

# Source the workspace  
source install/setup.bash  

# Launch LI-SLAM  
ros2 launch lidarslam lidarslam.launch.py  
```  

### **3. Launch FASTLIO2**  

```bash  
# Navigate to the FASTLIO2 package  
cd FASTLIOROS2  

# Launch FASTLIO2  
ros2 launch fast_lio mapping.launch.py  
```
> **Note:** It is recommended to run only one of the SLAM packages at a time.
### **4. Teleoperate the Drone**  

You can control the drone using either **xterm** or a **Gamepad**.  

#### Install xterm  

```bash  
sudo apt update  
sudo apt install xterm  
```  

#### Install RVIZ IMU Plugin  

```bash  
sudo apt install ros-humble-rviz-imu-plugin  
```  

---
## **People**
This project was lovingly made by Zaryan, Maleeha, and Shameer, all three of which go under the name of Zarmasha.

