# 🚀 CPS Project Log: LiDAR & RGB-D Sensor Setup

**Ouster OS1 • RPLIDAR A-Series • Orbbec RGB-D • ROS2 Jazzy • SLAM • Bag Recording**

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue) ![Python](https://img.shields.io/badge/Python-3.11-green) ![LiDAR](https://img.shields.io/badge/LiDAR-Ouster-orange)

---

##  Table of Contents
1. [3D LiDAR: Ouster OS1 Setup](#1-3d-lidar-ouster-os1-setup)  
2. [Troubleshooting Ouster LiDAR on Linux](#2-troubleshooting-ouster-lidar-on-linux)  
3. [2D LiDAR: RPLIDAR A-Series Setup](#3-2d-lidar-rplidar-a-series-setup)  
4. [Orbbec RGB-D Camera Setup (ROS2 Jazzy)](#4-orbbec-rgb-d-camera-setup-ros2-jazzy)  
5. [RGB-D Camera SLAM Challenge](#5-rgb-d-camera-slam-challenge)  
6. [LiDAR Bag Data Collection (ROS2)](#6-lidar-bag-data-collection-ros2)  
7. [RGB-D Camera SLAM Debugging & Bag Reconstruction](#7-rgb-d-camera-slam-debugging--bag-reconstruction)  
8. [Summary](#8-summary)  

---

# 1. 3D LiDAR: Ouster OS1 Setup
**Date:** 03.11.2025  

- **Objective:** Connect and visualize the Ouster OS1 3D LiDAR.  
- **Documentation:**
    - [Ouster SDK](https://github.com/ouster-lidar/ouster-sdk)  
    - [Ouster Studio](https://ouster.com/products/software/ouster-studio)  
- **Connection:** Direct Ethernet connection to mini PC.  
- **Outcome:** Sensor detected, firmware updated, visualized in Ouster Studio.

---

# 2. Troubleshooting Ouster LiDAR on Linux

### 2.1 Problem Diagnosis
**Symptoms:**
- `ouster-cli discover` finds the sensor (link-local IP, e.g., `169.254.41.35`)  
- `ouster-cli source <IP> viz` fails with `"Timeout was reached"`  
- After network config, `"No valid scans received"` appears  

**Root Causes:**
1. **Network Mismatch:** Sensor has a link-local IP but host Ethernet is on a different subnet.  
2. **Firewall Blocking:** UDP ports (7502/7503) blocked by host firewall.  

### 2.2 Solution: Step-by-Step

**Step 1: Identify Sensor IP & Ethernet Port**
```bash
ouster-cli discover   # find sensor IP
ip a                  # find Ethernet port (e.g., eno1)
```
**Step 2: Configure Host Network
``` bash
sudo ip addr add 169.254.41.100/16 dev eno1
```
***Step 3: Open Firewall Ports
```bash
sudo ufw allow 7502/udp
sudo ufw allow 7503/udp
```
Step 4: Connect to Sensor
```bash
ouster-cli source 169.254.41.35 viz
```
### Outcome:
- Connection established successfully, 3D point cloud visualized, firmware updated, Ouster Studio used.

# 3. 2D LiDAR: RPLIDAR A-Series Setup
Date: 03.11.2025

- Objective: Connect and visualize the RPLIDAR A-Series 2D LiDAR.
- Documentation: RPLIDAR Support

## 3.1 Challenge: Visualization
- Terminal only outputs raw serial data; no ROS2 or RViz visualization initially.

## 3.2 Workaround: Python & Matplotlib
- Custom Python script reads serial data and visualizes 2D scans using matplotlib.

## 3.3 Identifying the Correct Serial Port
```bash
ls /dev/ttyUSB*
# Output: /dev/ttyUSB0
```
## Outcome
- Python script successfully reads scan data and visualizes 2D surroundings.

# 4. Orbbec RGB-D Camera Setup (ROS2 Jazzy)
Date: 11.11.2025

##4.1 Hardware Setup
- Connected Orbbec RGB-D camera via USB-C.
- Device recognized successfully.

## 4.2 Software & Driver Setup
```bash
cd ~/ros2_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git

cd ~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger

cd ~/ros2_ws
colcon build --packages-select orbbec_camera --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/ros2_ws/install/setup.bash
```
Launch Camera Node Example:
```bash
ros2 launch orbbec_camera gemini_330_series.launch.py
```
## 4.3 Visualization in RViz
- Add Image display for RGB stream (/camera/color/image_raw)
- Add PointCloud2 display for depth (/camera/depth/points)
## Outcome
- Live RGB + depth feed visualized; pipeline stable.

# 5. RGB-D Camera SLAM Challenge
Date: 12.11.2025

## 5.1 Objective
- Run SLAM using RGB-D feed to build 3D maps in RViz.

## 5.2 Challenges
- SLAM node failed on first launch due to package issues.
- Mini PC performance limited compilation; parallel builds caused crashes.

## 5.3 Resolution
- Reinstalled SLAM packages sequentially (colcon build)
- Verified dependencies: RTAB-Map, ORB-SLAM3

## 5.4 Example Launch Command
```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  rtabmap_args:="--delete_db_on_start" \
  frame_id:=camera_link \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  approx_sync:=false \
  rtabmapviz:=false
```
## Outcome 
- Live RGB-D feed and point cloud visualized; SLAM pipeline verified.

# 6. LiDAR Bag Data Collection (ROS2)
Date: 13.11.2025

## 6.1 Objective
- Record Ouster LiDAR data as ROS2 bags for SLAM and sensor-fusion tests.

## 6.2 Initial Problems
- No messages on /ouster/points
- RViz2 QoS warnings
- Wrong ROS2 driver repository

## 6.3 Fix: Correct ROS2 Driver
```bash
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git

cd ~/ros2_ws
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
## 6.4 Record a Bag
```bash
ros2 bag record /ouster/points
```
## 6.5 Playback
```bash
ros2 bag play <bag_folder_name>
```
- Open RViz2
- Add PointCloud2 display on /ouster/points

## Outcome
- Bag playback works; sensor not required for testing SLAM pipelines.

# 7. RGB-D Camera SLAM Debugging & Bag Reconstruction
Date: 13.11.2025

##7.1 Initial Problem
Odometry node crashed due to RGB-depth resolution mismatch:
```Scale-MissMatch
RGB: 1280x720
Depth: 640x576
```
## 7.2 Fix
```bash
ros2 launch orbbec_camera femto_mega.launch.py depth_registration:=true
- Aligns depth frame to RGB intrinsics
- Verified streams:
ros2 topic echo /camera/color/image_raw --once
ros2 topic echo /camera/depth/image_raw --once
```
## 7.3 Record New Bag
```bash
ros2 bag record \
  /camera/color/image_raw \
  /camera/depth/image_raw \
  /camera/color/camera_info \
  /tf \
  /tf_static
```
## 7.4 Run SLAM
**Terminal 1: Play bag
```bash
ros2 bag play <new_bag_name> --clock --loop
```
**Terminal 2: Launch RTAB-Map
```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  use_sim_time:=true \
  rtabmap_args:="--delete_db_on_start" \
  frame_id:=camera_link \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  approx_sync:=true \
  rtabmapviz:=true
```
## Outcome
- SLAM pipeline works; synchronized RGB-D frames, trajectory, and 3D point cloud displayed.
