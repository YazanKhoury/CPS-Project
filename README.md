<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Robotics Perception & Mapping Report</title>
  <link href="https://fonts.googleapis.com/css2?family=Roboto:wght@400;700&display=swap" rel="stylesheet">
  <style>
    body {
      font-family: 'Roboto', sans-serif;
      margin: 0;
      padding: 0;
      background: #f5f5f5;
      color: #333;
      line-height: 1.6;
    }
    header {
      background: linear-gradient(90deg, #4a90e2, #50e3c2);
      color: white;
      padding: 2rem 1rem;
      text-align: center;
    }
    header h1 {
      margin: 0;
      font-size: 2rem;
    }
    header p {
      font-size: 1.1rem;
      margin-top: 0.5rem;
    }
    .container {
      max-width: 1000px;
      margin: 2rem auto;
      padding: 0 1rem;
    }
    h2 {
      color: #4a90e2;
      margin-top: 2rem;
    }
    h3 {
      color: #50e3c2;
      margin-top: 1rem;
    }
    pre {
      background: #272822;
      color: #f8f8f2;
      padding: 1rem;
      overflow-x: auto;
      border-radius: 5px;
    }
    code {
      font-family: monospace;
    }
    .badge {
      display: inline-block;
      background: #ff6f61;
      color: white;
      padding: 0.3rem 0.6rem;
      border-radius: 5px;
      font-size: 0.8rem;
      margin-right: 0.5rem;
      margin-bottom: 0.5rem;
    }
    .toc {
      background: #e3f2fd;
      padding: 1rem;
      border-left: 4px solid #4a90e2;
      margin-bottom: 2rem;
    }
    .toc ul {
      padding-left: 1rem;
    }
    .toc a {
      text-decoration: none;
      color: #333;
    }
    .phase {
      background: #fff;
      padding: 1rem 1.5rem;
      border-radius: 8px;
      margin-bottom: 1.5rem;
      box-shadow: 0 4px 8px rgba(0,0,0,0.05);
    }
  </style>
</head>
<body>

<header>
  <h1>🚀 Robotics Perception & Mapping Report</h1>
  <p>Ouster LiDAR • RPLIDAR • Orbbec RGB-D • SLAM • Bag Recording</p>
</header>

<div class="container">
  <div class="toc">
    <h3>Table of Contents</h3>
    <ul>
      <li><a href="#phase1">Phase 1: Ouster OS1 LiDAR Setup</a></li>
      <li><a href="#phase2">Phase 2: RPLIDAR A-Series Setup</a></li>
      <li><a href="#phase3">Phase 3: Orbbec RGB-D Camera (ROS2 Jazzy)</a></li>
      <li><a href="#phase4">Phase 4: SLAM, Debugging & Bag Recording</a></li>
      <li><a href="#phase5">Phase 5: Ouster LiDAR Bag Recording</a></li>
    </ul>
  </div>

  <div id="phase1" class="phase">
    <h2>Phase 1: Ouster OS1 LiDAR Setup</h2>
    <p><strong>Date:</strong> 03 November 2025</p>
    <h3>1.1 Objective</h3>
    <p>Connect the Ouster OS1 LiDAR over Ethernet and visualize real-time 3D point clouds.</p>
    <h3>1.2 Network Detection</h3>
    <pre><code>ouster-cli discover</code></pre>
    <p>This displays the LiDAR’s IP address (usually 169.254.x.x).</p>
    <h3>1.3 Network Configuration</h3>
    <pre><code>sudo ip addr add 169.254.41.100/16 dev eno1</code></pre>
    <h3>1.4 Firewall Configuration</h3>
    <pre><code>sudo ufw allow 7502/udp
sudo ufw allow 7503/udp</code></pre>
    <h3>1.5 Visualizing the Sensor</h3>
    <pre><code>ouster-cli source 169.254.41.35 viz</code></pre>
    <h3>1.6 Outcome</h3>
    <p>Successful connection, firmware updated, and real-time point cloud shown in Ouster Studio.</p>
  </div>

  <div id="phase2" class="phase">
    <h2>Phase 2: RPLIDAR A-Series Setup</h2>
    <p><strong>Date:</strong> 03 November 2025</p>
    <h3>2.1 Device Detection</h3>
    <pre><code>ls /dev/ttyUSB*</code></pre>
    <p>Example: /dev/ttyUSB0</p>
    <h3>2.2 Visualization Challenge</h3>
    <p>The sensor outputs raw serial data but offers no built-in visualization.</p>
    <h3>2.3 Solution — Custom Python Visualizer</h3>
    <p>A 2D scan visualizer was written using Python + Matplotlib to simulate real-time LiDAR output.</p>
    <h3>2.4 Outcome</h3>
    <p>Sensor connected and real-time 2D scan display succeeded.</p>
  </div>

  <div id="phase3" class="phase">
    <h2>Phase 3: Orbbec RGB-D Camera (ROS2 Jazzy)</h2>
    <p><strong>Date:</strong> 11 November 2025</p>
    <h3>3.1 Objective</h3>
    <p>Build and launch the Orbbec Femto Mega RGB-D camera driver on ROS 2.</p>
    <h3>3.2 Cloning the Driver</h3>
    <pre><code>cd ~/ros2_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git</code></pre>
    <h3>3.3 Installing Udev Rules</h3>
    <pre><code>cd ~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger</code></pre>
    <h3>3.4 Building the Workspace</h3>
    <pre><code>cd ~/ros2_ws
colcon build --packages-select orbbec_camera --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash</code></pre>
    <h3>3.5 Launching the Camera</h3>
    <pre><code>ros2 launch orbbec_camera femto_mega.launch.py</code></pre>
    <h3>3.6 Visualizing in RViz</h3>
    <ul>
      <li>RGB Stream — Topic: /camera/color/image_raw</li>
      <li>Depth Point Cloud — Topic: /camera/depth/points</li>
    </ul>
    <h3>3.7 Outcome</h3>
    <p>RGB + depth streaming fully functional. Point cloud visible in RViz2.</p>
  </div>

  <div id="phase4" class="phase">
    <h2>Phase 4: SLAM, Debugging & Bag Recording</h2>
    <p><strong>Date:</strong> 12–13 November 2025</p>
    <h3>4.1 Issue: SLAM Not Working</h3>
    <p>RTAB-Map failed due to mismatched image sizes:</p>
    <pre><code>[rgb=1280x720 depth=640x576]</code></pre>
    <h3>4.2 Fix — Enable Depth Registration</h3>
    <pre><code>ros2 launch orbbec_camera femto_mega.launch.py depth_registration:=true</code></pre>
    <h3>4.3 Verify Streams</h3>
    <pre><code>ros2 topic echo /camera/color/image_raw --once
ros2 topic echo /camera/depth/image_raw --once</code></pre>
    <h3>4.4 Recording a ROS2 Bag</h3>
    <pre><code>ros2 bag record \
/camera/color/image_raw \
/camera/depth/image_raw \
/camera/color/camera_info \
/tf \
/tf_static</code></pre>
    <h3>4.5 SLAM Using Bag File</h3>
    <pre><code>Terminal 1 — Play Bag
ros2 bag play &lt;bag_name&gt; --clock --loop

Terminal 2 — Launch SLAM
ros2 launch rtabmap_launch rtabmap.launch.py \
use_sim_time:=true \
rtabmap_args:="--delete_db_on_start" \
frame_id:=camera_link \
rgb_topic:=/camera/color/image_raw \
depth_topic:=/camera/depth/image_raw \
camera_info_topic:=/camera/color/camera_info \
approx_sync:=true \
rtabmapviz:=true</code></pre>
    <h3>4.6 Outcome</h3>
    <p>Odometry successfully corrected. SLAM map generated. Point cloud and trajectory visible.</p>
  </div>

  <div id="phase5" class="phase">
    <h2>Phase 5: Ouster LiDAR Bag Recording</h2>
    <p><strong>Date:</strong> 13 November 2025</p>
    <h3>5.1 Issue</h3>
    <p>Wrong repository used — /ouster/points not publishing.</p>
    <h3>5.2 Fix — Correct ROS2 Driver</h3>
    <pre><code>cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git

cd ~/ros2_ws
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash</code></pre>
    <h3>5.3 Record LiDAR Data</h3>
    <pre><code>ros2 bag record /ouster/points</code></pre>
    <h3>5.4 Playback</h3>
    <pre><code>ros2 bag play &lt;bag&gt;

Visualize in RViz: Add PointCloud2, Select topic: /ouster/points</code></pre>
    <h3>5.5 Outcome</h3>
    <p>Bag replay functional. Visualized LiDAR without hardware.</p>
  </div>

</div>

</body>
</html>
