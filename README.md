# 🚀 CPS Project Log: LiDAR & RGB-D Sensor Setup

**Ouster OS1 • RPLIDAR A-Series • Orbbec RGB-D • ROS2 Jazzy • SLAM • Bag Recording**

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue) ![Python](https://img.shields.io/badge/Python-3.11-green) ![LiDAR](https://img.shields.io/badge/LiDAR-Ouster-orange)

---

## 📌 Table of Contents
1. [3D LiDAR: Ouster OS1 Setup](#1-3d-lidar-ouster-os1-setup)  
2. [Troubleshooting Ouster LiDAR on Linux](#2-troubleshooting-ouster-lidar-on-linux)  
3. [2D LiDAR: RPLIDAR A-Series Setup](#3-2d-lidar-rplidar-a-series-setup)  
4. [Orbbec RGB-D Camera Setup (ROS2 Jazzy)](#4-orbbec-rgb-d-camera-setup-ros2-jazzy)  
5. [RGB-D Camera SLAM Challenge](#5-rgb-d-camera-slam-challenge)  
6. [LiDAR Bag Data Collection (ROS2)](#6-lidar-bag-data-collection-ros2)  
7. [RGB-D Camera SLAM Debugging & Bag Reconstruction](#7-rgb-d-camera-slam-debugging--bag-reconstruction)  
8. [Summary](#8-summary)  

---

## 1. 3D LiDAR: Ouster OS1 Setup
**Date:** 03.11.2025  

- **Objective:** Connect and visualize the Ouster OS1 3D LiDAR.  
- **Documentation:**
    - [Ouster SDK](https://github.com/ouster-lidar/ouster-sdk)  
    - [Ouster Studio](https://ouster.com/products/software/ouster-studio)  
- **Connection:** Direct Ethernet connection to mini PC.  
- **Outcome:** Sensor detected, firmware updated, visualized in Ouster Studio.

---

## 2. Troubleshooting Ouster LiDAR on Linux

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
###Outcome:
- Connection established successfully, 3D point cloud visualized, firmware updated, Ouster Studio used.
