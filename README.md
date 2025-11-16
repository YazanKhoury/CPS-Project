# Robotics Perception & Mapping Report

**Ouster LiDAR • RPLIDAR • Orbbec RGB-D • SLAM • Bag Recording**  

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue) ![Python](https://img.shields.io/badge/Python-3.11-green) ![LiDAR](https://img.shields.io/badge/LiDAR-Ouster-orange)

---

## Table of Contents
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

```
### 1.3 Network Configuration
```bash
sudo ip addr add 169.254.41.100/16 dev eno1

```
### 1.4 Firewall Configuration
```bash
sudo ufw allow 7502/udp
sudo ufw allow 7503/udp

```
### 1.5 Visualizing the Sensor
```bash
ouster-cli source 169.254.41.35 viz
```
### 1.6 Outcome
- Successful connection
- Firmware Update
- Real-Time point cloud shown in Ouster Studio

# Phase 2: RPLIDAR A-Series Setup

Date: 03 November 2025

### 2.1 Device Detection
```bash
ls /dev/ttyUSB*
Example: /dev/ttyUSB0
```

### 2.2 Visualization Challenge

- The sensor outputs raw serial data but offers no built-in visualization.

### 2.3 Solution — Custom Python Visualizer

- A 2D scan visualizer was written using Python + Matplotlib to simulate real-time LiDAR output.

### 2.4 Outcome

- Sensor connected
- Real-time 2D scan display succeeded

# Phase 3: Orbbec RGB-D Camera (ROS2 Jazzy)

Date: 11 November 2025

## 3.1 Objective

Build and launch the Orbbec Femto Mega RGB-D camera driver on ROS 2.

### 3.2 Cloning the Driver
``` bash
cd ~/ros2_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
```
### 3.3 Installing Udev Rules
``` bash
cd ~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 3.4 Building the Workspace
```bash
cd ~/ros2_ws
colcon build --packages-select orbbec_camera --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
### 3.5 Launching the Camera
```bash
ros2 launch orbbec_camera femto_mega.launch.py
```
### 3.6 Visualizing in RViz

- RGB Stream — Topic: /camera/color/image_raw
- Depth Point Cloud — Topic: /camera/depth/points

## 3.7 Outcome

- RGB + depth streaming fully functional
- Point cloud visible in RViz2

# Phase 4: SLAM, Debugging & Bag Recording

Date: 12–13 November 2025

## 4.1 Issue: SLAM Not Working

- RTAB-Map failed due to mismatched image sizes:
```bash
[rgb=1280x720 depth=640x576]
```
## 4.2 Fix — Enable Depth Registration
```bash
ros2 launch orbbec_camera femto_mega.launch.py depth_registration:=true


This forces both RGB + depth to match at 1280×720.

4.3 Verify Streams
ros2 topic echo /camera/color/image_raw --once
ros2 topic echo /camera/depth/image_raw --once

4.4 Recording a ROS2 Bag
ros2 bag record \
/camera/color/image_raw \
/camera/depth/image_raw \
/camera/color/camera_info \
/tf \
/tf_static

4.5 SLAM Using Bag File

Terminal 1 — Play Bag

ros2 bag play <bag_name> --clock --loop


Terminal 2 — Launch SLAM

ros2 launch rtabmap_launch rtabmap.launch.py \
use_sim_time:=true \
rtabmap_args:="--delete_db_on_start" \
frame_id:=camera_link \
rgb_topic:=/camera/color/image_raw \
depth_topic:=/camera/depth/image_raw \
camera_info_topic:=/camera/color/camera_info \
approx_sync:=true \
rtabmapviz:=true

4.6 Outcome

Odometry successfully corrected

SLAM map generated

Point cloud and trajectory visible
