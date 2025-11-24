# 2024 TurtleBot3 Autonomous Driving Robot

An autonomous driving robot project based on TurtleBot3. It performs various missions using Qt5 GUI and OpenCV in a ROS2 Humble environment.

## Table of Contents
- [Overview](#overview)
- [System Requirements](#system-requirements)
- [Package Structure](#package-structure)
- [Installation](#installation)
- [Usage](#usage)
- [Mission Description](#mission-description)
- [License](#license)

## Overview

This project is an autonomous driving robot system using TurtleBot3, performing various missions including traffic light recognition, intersection driving, construction zone avoidance, parking, and gate bar passing based on line tracing.

### Key Features
- **Line Tracing**: Yellow/white line detection based on RANSAC algorithm
- **Perspective Transform**: Accurate line recognition through Bird's Eye View
- **HSV Color Detection**: Object recognition for traffic lights, signs, gate bars, etc.
- **IMU-based Control**: Precise direction control using relative angles
- **Qt5 GUI**: Real-time video monitoring and parameter adjustment

## System Requirements

| Component | Version |
|-----------|---------|
| Ubuntu | 22.04 LTS |
| ROS2 | Humble |
| Qt | 5.x |
| OpenCV | 4.x |
| Boost | 1.71.0+ |

### Hardware
- TurtleBot3 (Burger/Waffle)
- USB Camera (UDP streaming)
- PSD Sensors (3 channels)
- IMU Sensor

## Package Structure
```
2024_turtlebot/
├── robit_master/       # Main control node
│   ├── include/        # Header files
│   ├── src/            # Source code
│   └── ui/             # Qt UI files
├── robit_vision/       # Vision processing node
│   ├── include/        # Header files
│   ├── src/            # Source code
│   ├── ui/             # Qt UI files
│   └── txt_data/       # Parameter data
├── robit_msgs/         # Custom messages
│   └── msg/            # Message definition files
├── set_imu/            # IMU processing node
│   ├── include/
│   └── src/
└── turtle_go/          # Launcher GUI
    ├── include/
    ├── src/
    └── launch/
```

### Package Description

| Package | Description |
|---------|-------------|
| `robit_master` | Robot control logic, line tracing, mission execution |
| `robit_vision` | Image processing, object detection, line extraction |
| `robit_msgs` | VisionMsg, MasterMsg, SimpleMoveMsg definitions |
| `set_imu` | IMU data processing and relative angle calculation |
| `turtle_go` | System launcher |

## Installation

### 1. Set up ROS2 Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone Repository
```bash
git clone https://github.com/mkdir-sweetiepie/2024_turtlebot.git
```

### 3. Install Dependencies
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build
```bash
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Method 1: Using Launch File
```bash
ros2 launch turtle_go turtle-go.launch.py
```

### Method 2: Using GUI Launcher
```bash
ros2 run turtle_go turtle_go
```
Click the **On** button in the GUI to run all nodes.

### Method 3: Running Individual Nodes
```bash
# Terminal 1: IMU node
ros2 run set_imu set_imu

# Terminal 2: Vision node
ros2 run robit_vision robit_vision

# Terminal 3: Master node
ros2 run robit_master robit_master
```

## Mission Description

| Mission | Description |
|---------|-------------|
| **Traffic** | Start after detecting green traffic light |
| **Cross** | Determine left/right turn by recognizing blue signs |
| **Construct** | Avoid obstacles using PSD sensors in construction zones |
| **Parking** | Park in designated parking space |
| **Zigzag** | Line tracing through zigzag section |
| **Gatebar** | Detect and pass through gate bar |

### Mission Order Configuration
You can set the mission order and starting IMU angles in the mission table of the `robit_vision` GUI.

## Topic Structure

### Published Topics
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Robot velocity command |
| `/turtle_vision` | robit_msgs/VisionMsg | Vision processing results |
| `/turtle_master` | robit_msgs/MasterMsg | Mission status |
| `/set_imu` | std_msgs/UInt32 | Processed IMU angle |

### Subscribed Topics
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/imu` | sensor_msgs/Imu | IMU sensor data |
| `/robit_psd` | std_msgs/UInt16MultiArray | PSD sensor data |
| `/robit_button` | std_msgs/Int8MultiArray | Button input |

## Parameter Adjustment

### HSV Color Values
You can adjust HSV values in real-time for the following items in the `robit_vision` GUI:
- Yellow (yellow line)
- White (white line)
- Gatebar (gate bar)
- Traffic (traffic light)
- Bluesign (blue sign)

Parameters are saved in `txt_data/parameter_data.txt`.
