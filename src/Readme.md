# Capture System (ROS 2)

This repository contains a ROS 2 Humble-based synchronized image capture system for industrial cameras, developed for research and development in computer vision and robotics. The system is designed for UAV applications, specifically implementing an omnidirectional surveillance strategy that allows a single camera to achieve a $360^{\circ}$ field of view (FoV) through synchronized rotation and capture.



## 📂 Repository Structure

* **`ros2_hik_camera`**: Base ROS driver and interface for Hikvision (MVS) industrial cameras.
* **`spe_degree_capture`**: Core logic node that triggers captures at specific angular intervals.
* **`sim_rotate.py`**: A Python-based simulator that publishes simulated yaw data at high frequencies to test the trigger logic.

## 🛠️ Requirements

### 1. Hardware
* **Camera**: Hikvision Industrial Camera (Global Shutter is highly recommended).
* **Compute**: N305 or similar onboard computer running Ubuntu 22.04.
* **Interface**: USB 3.0 or GigE connection for high-speed data transfer.

### 2. Software
* **OS**: Ubuntu 22.04 + ROS 2 Humble.
* **SDK**: Hikvision MVS SDK (Machine Vision Software).
* **Dependencies**: OpenCV 4.x, `rclcpp`, `std_msgs`, C++17.

## 🚀 Installation & Build

```bash
# Create and navigate to the workspace
mkdir -p ~/hik_camera_ws/src
cd ~/hik_camera_ws/src

# Clone the repository and build
cd ~/hik_camera_ws
colcon build --packages-select ros2_hik_camera spe_degree_capture

# Source the environment
source install/setup.bash
```

## 🎮 Operational WorkflowTerminal 

1: Launch Capture NodeInitializes hardware and sets the standby trigger state.Bash
```bash
ros2 run spe_degree_capture capture_node
```

2: Start Rotation SimulatorSimulates UAV rotation (e.g., $25~rad/s$) to provide orientation feedback.Bash
```bash
python3 sim_rotate.py
```


## 📺 Visualization in RViz2

To visualize the real-time camera feed alongside other robotic data (like LiDAR point clouds from FAST-LIO2), follow these steps.

### 1. Launch the Hikvision Camera Driver
This node publishes the raw image stream to the `/camera/image_raw` topic.
```bash
# Make sure you have sourced your workspace
source ~/hik_camera_ws/install/setup.bash

# Launch the driver (adjust the package name if different)
ros2 run ros2_hik_camera hik_camera_node

ros2 run rviz2 rviz2
```
