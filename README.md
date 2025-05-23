# 🌲 AURORA – Multi-UAV SAR System with LI-SLAM & FASTLIO2

Welcome to **AURORA** – a modular, simulation-ready, multi-UAV system for **Search and Rescue (SAR)** in complex **forest environments**. Developed at **Habib University** as a final-year Capstone Project, AURORA integrates cutting-edge SLAM, path planning, and human detection technologies under one unified framework.

Built with **ROS2 (Humble)**, **Gazebo Classic**, and real-time LiDAR-thermal SLAM pipelines, AURORA is purpose-designed for GPS-denied, under-canopy scenarios where traditional SAR efforts struggle.

---

## 📺 Demo Video

See AURORA in action – live mapping, autonomous flight, and human detection in a simulated dense forest:

[![Demo Video](Thumbnail.png)](https://youtu.be/zFVc0A3nOic)

---

## 📖 Table of Contents

- [Overview](#overview)
- [Key Contributions](#key-contributions)
- [System Features](#system-features)
- [Architecture Overview](#architecture-overview)
- [Key Packages](#key-packages)
- [Acknowledgements](#acknowledgements)
- [Getting Started](#getting-started)
- [Installation](#installation)
- [Usage](#usage)
- [Changing Gazebo Worlds](#changing-gazebo-worlds)
- [People](#people)

---

## 🧭 Overview

**AURORA** addresses the core challenges of forest SAR operations:

- 🚫 GPS-denied conditions  
- 🌲 Occlusion from canopy and dense vegetation  
- ⚠️ Visual aliasing and unreliable communication

Our system leverages **LiDAR, IMU, and thermal cameras** mounted on UAVs to enable:

- Real-time **Simultaneous Localization and Mapping (SLAM)**  
- **Thermal-aware exploration** and victim detection  
- Multi-agent **autonomous navigation** and **teleoperation** support

---

## 🚀 Key Contributions

- ✅ First complete benchmarked SAR pipeline for forest environments  
- 🔁 Evaluation of **LI-SLAM** vs. **FASTLIO2** under identical testbeds  
- 🔥 Integration of thermal human detection using YOLOv8  
- 📡 Autonomous and operator-assisted mission architecture  
- 📊 Field-oriented performance benchmarking (exploration speed, SLAM RMSE, detection accuracy)

---

## ⚙️ System Features

- 🛰️ Multi-UAV, ROS2-based architecture  
- 🗺️ Real-time mapping using LiDAR-Inertial odometry  
- 🔍 Frontier and probability-map exploration algorithms  
- 🔥 YOLOv8-based thermal human detection  
- 🧭 Path planning with RRT/bi-RRT + APF for collision-free travel  
- 📍 Victim localization with ≤5m accuracy

---

## 🧱 Architecture Overview

**Base Station:**
- Mission planning
- Operator interface (RViz2)
- Real-time visualization

**UAV Stack:**
- SLAM: LI-SLAM / FASTLIO2
- Exploration logic
- Target detection module (YOLO + Thermal)
- 3D Path Planner
- Return-to-base behavior

---

## 📦 Key Packages

1. **SJTU Drone** – UAV model, control interfaces, and world setup  
2. **LI-SLAM** – Pose-graph based SLAM for dense environments  
3. **FASTLIO2** – Real-time, tightly coupled LiDAR-Inertial odometry  
4. **Exploration Module** – Frontier, thermal, and probability-based logic  
5. **Human Detection** – YOLOv8, trained on thermal forest imagery

---

## 🙏 Acknowledgements

- [SJTU Drone (Parrot AR2)](https://github.com/NovoG93/sjtu_drone) – by Shanghai Jiao Tong University  
- [LI-SLAM ROS2](https://github.com/rsasaki0109/li_slam_ros2) – by Ryuhei Sasaki  
- [FASTLIO2 ROS2](https://github.com/hku-mars/FAST_LIO/tree/ROS2) – by HKU-MARS Lab  
- Advised by **Dr. Abdul Basit Memon**, Habib University

---

## 🛠️ Getting Started

### Prerequisites

- ROS2 Humble
- Gazebo Classic 11
- Python 3.8+
- C++ compiler (for ROS nodes)
- [Gazebo Models Download (Google Drive)](https://drive.google.com/drive/folders/1m94-q4EyJTxHPq3p_1vGZ0i0HHRN0Vhf)

### Clone the Repository

```bash
git clone https://github.com/ShameerMasroor/LI-SLAM-and-FASTLIO2-for-Teleoperated-UAVs.git
