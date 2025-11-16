# 🚀 Robotics Perception & Mapping Report

**Ouster LiDAR • RPLIDAR • Orbbec RGB-D • SLAM • Bag Recording**  

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue) ![Python](https://img.shields.io/badge/Python-3.11-green) ![LiDAR](https://img.shields.io/badge/LiDAR-Ouster-orange)

---

## 📌 Table of Contents
1. [Phase 1: Ouster OS1 LiDAR Setup](#phase-1-ouster-os1-lidar-setup)  
2. [Phase 2: RPLIDAR A-Series Setup](#phase-2-rplidar-a-series-setup)  
3. [Phase 3: Orbbec RGB-D Camera (ROS2 Jazzy)](#phase-3-orbbec-rgb-d-camera-ros2-jazzy)  
4. [Phase 4: SLAM, Debugging & Bag Recording](#phase-4-slam-debugging--bag-recording)  
5. [Phase 5: Ouster LiDAR Bag Recording](#phase-5-ouster-lidar-bag-recording)  

---

## Phase 1: Ouster OS1 LiDAR Setup
**Date:** 03 November 2025

### 1.1 Objective
Connect the Ouster OS1 LiDAR over Ethernet and visualize real-time 3D point clouds.

### 1.2 Network Detection
```bash
ouster-cli discover

1.3 Network Configuration
