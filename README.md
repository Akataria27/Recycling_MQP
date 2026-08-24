# Recycling_MQP

## Overview
This project implements a complete control and grasping system for the Franka Emika Panda robot using ROS 2 Humble and MoveIt2. It includes simulation capabilities, motion planning, hardware control, object detection, and an integrated grasping pipeline with perception, planning, and execution modules.

---

## Table of Contents
- [Project Structure](#project-structure)
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Setup Instructions](#setup-instructions)
- [Usage](#usage)
- [Grasping Pipeline](#grasping-pipeline)
- [Project Components](#project-components)
- [Troubleshooting](#troubleshooting)

---

## Project Structure

```
RBE595/
├── src/
│   ├── franka_arm_ros2/                # Official Franka ROS 2 integration
│   │   ├── franka_bringup/             # Launch and configuration for robot bring-up
│   │   │   ├── config/
│   │   │   │   ├── controllers.yaml    # Joint trajectory and gripper controllers
│   │   │   │   └── dual_controllers.yaml # Dual arm configuration
│   │   │   └── launch/
│   │   │       ├── franka.launch.py    # Single arm launch
│   │   │       ├── dual_franka.launch.py # Dual arm launch
│   │   │       ├── move_to_start_example_controller.launch.py
│   │   │       ├── joint_impedance_example_controller.launch.py
│   │   │       ├── gravity_compensation_example_controller.launch.py
│   │   │       └── joint_velocity_example_controller.launch.py
│   │   │
│   │   ├── franka_control2/            
│   │   │   ├── src/
│   │   │   │   └── franka_control2_node.cpp # Main control node
│   │   │   ├── CMakeLists.txt
│   │   │   └── package.xml
│   │   │   
│   │   ├── franka_description/         # Robot URDF and configuration
│   │   │   ├── urdf/
│   │   │   │   ├── panda.urdf.xacro    # Main robot URDF (arm + gripper)
│   │   │   │   ├── panda_arm.xacro     # Arm kinematic chain
│   │   │   │   ├── hand.xacro          # Gripper definition
│   │   │   │   └── panda.ros2_control.xacro # ros2_control hardware interface
│   │   │   ├── robots/                 # Complete robot descriptions
│   │   │   ├── meshes/                 # Robot mesh files for visualization
│   │   │   ├── rviz/                   # RViz configuration files
│   │   │   ├── launch/
│   │   │   │   └── visualize_franka.launch.py # RViz visualization launch
│   │   │   ├── test/
│   │   │   │   └── urdf_tests.py       # URDF validation tests
│   │   │   ├── CMakeLists.txt
│   │   │   └── package.xml
│   │   │
│   │   ├── franka_moveit_config/       # MoveIt2 configuration
│   │   │   ├── config/
│   │   │   │   ├── controllers.yaml    # MoveIt controller interface
│   │   │   │   ├── panda_controllers.yaml
│   │   │   │   ├── panda_ros_controllers.yaml
│   │   │   │   ├── dual_panda_controllers.yaml
│   │   │   │   ├── dual_panda_ros_controllers.yaml
│   │   │   │   ├── panda_mock_ros_controllers.yaml
│   │   │   │   ├── kinematics.yaml     # IK solver configuration
│   │   │   │   └── ompl_planning.yaml  # OMPL planner configuration
│   │   │   ├── launch/
│   │   │   │   ├── moveit.launch.py    # Single arm MoveIt launch
│   │   │   │   ├── dual_moveit.launch.py # Dual arm MoveIt launch
│   │   │   │   └── moveit_real_arm_platform.launch.py # Real hardware launch
│   │   │   ├── srdf/
│   │   │   │   └── panda.srdf.xacro    # Semantic robot description
│   │   │   ├── rviz/                   # MoveIt RViz configuration
│   │   │   ├── test/
│   │   │   │   └── srdf_tests.py       # SRDF validation tests
│   │   │   ├── CMakeLists.txt
│   │   │   └── package.xml
│   │   │
│   │   ├── franka_hardware/            # Low-level hardware interface (C++)
│   │   │   ├── src/
│   │   │   │   ├── franka_hardware_interface.cpp # Main hardware interface
│   │   │   │   ├── franka_multi_hardware_interface.cpp # Multi-arm support
│   │   │   │   ├── robot.cpp           # Robot state management
│   │   │   │   ├── control_mode.cpp    # Control mode handling
│   │   │   │   ├── helper_functions.cpp
│   │   │   │   ├── franka_executor.cpp # Command execution
│   │   │   │   ├── franka_param_service_server.cpp
│   │   │   │   └── franka_error_recovery_service_server.cpp
│   │   │   ├── include/
│   │   │   ├── franka_hardware.xml     # Hardware plugin definition
│   │   │   ├── CMakeLists.txt
│   │   │   └── package.xml
│   │   │
│   │   ├── franka_gripper/             # Gripper control
│   │   │   ├── src/
│   │   │   │   └── gripper_action_server.cpp
│   │   │   ├── scripts/
│   │   │   │   └── fake_gripper_state_publisher.py # Fake gripper for simulation
│   │   │   ├── config/
│   │   │   │   └── franka_gripper_node.yaml
│   │   │   ├── launch/
│   │   │   │   └── gripper.launch.py
│   │   │   ├── CMakeLists.txt
│   │   │   └── package.xml
│   │   │
│   │   ├── franka_example_controllers/ # Example controller implementations (C++)
│   │   │   ├── src/
│   │   │   │   ├── joint_position_example_controller.cpp
│   │   │   │   ├── joint_velocity_example_controller.cpp
│   │   │   │   ├── joint_impedance_example_controller.cpp
│   │   │   │   ├── dual_joint_impedance_example_controller.cpp
│   │   │   │   ├── dual_joint_velocity_example_controller.cpp
│   │   │   │   ├── cartesian_velocity_example_controller.cpp
│   │   │   │   ├── gravity_compensation_example_controller.cpp
│   │   │   │   ├── move_to_start_example_controller.cpp
│   │   │   │   └── motion_generator.cpp
│   │   │   ├── franka_example_controllers.xml # Plugin definition
│   │   │   ├── CMakeLists.txt
│   │   │   └── package.xml
│   │   │
│   │   ├── franka_robot_state_broadcaster/ # State publishing
│   │   │   ├── src/
│   │   │   │   ├── franka_robot_state_broadcaster.cpp
│   │   │   │   └── franka_robot_state_broadcaster_parameters.yaml
│   │   │   ├── franka_robot_state_broadcaster.xml
│   │   │   ├── CMakeLists.txt
│   │   │   └── package.xml
│   │   │
│   │   ├── franka_semantic_components/  # Semantic component wrappers
│   │   │   ├── src/
│   │   │   │   ├── franka_robot_state.cpp
│   │   │   │   └── franka_robot_model.cpp
│   │   │   ├── include/
│   │   │   ├── CMakeLists.txt
│   │   │   └── package.xml
│   │   │
│   │   ├── franka_msgs/                # Custom message definitions
│   │   │   ├── msg/
│   │   │   │   ├── FrankaState.msg     # Robot state message
│   │   │   │   ├── Errors.msg          # Error definitions
│   │   │   │   └── GraspEpsilon.msg    # Gripper epsilon parameter
│   │   │   ├── srv/
│   │   │   ├── action/
│   │   │   ├── CMakeLists.txt
│   │   │   └── package.xml
│   │   │
│   │   ├── README.md
│   │   ├── LICENSE
│   │   ├── CHANGELOG.md
│   │   ├── Dockerfile
│   │   └── Jenkinsfile
│   │
│   ├── camera_module/                  # Vision and inference
│   │   ├── camera_module/
│   │   │   ├── __init__.py
│   │   │   ├── camera.py               # Camera interface
│   │   │   ├── camera_node.py          # ROS 2 camera node
│   │   │   ├── infer.py                # Inference pipeline
│   │   │   ├── infer_live.py           # Live inference
│   │   │   ├── calibrate_camera.py     # Camera calibration
│   │   │   ├── classify_server.py      # Classification service
│   │   │   └── train.py                # Model training
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   ├── resource/
│   │   ├── test/
│   │   │   ├── test_copyright.py
│   │   │   ├── test_flake8.py
│   │   │   └── test_pep257.py
│   │   ├── "Commands for camera module" # Documentation file
│   │   └── README.md (if present)
│   │
│   ├── camera_interfaces/              # Camera message definitions
│   │   ├── msg/
│   │   │   └── BoundingBox.msg         # Object bounding box
│   │   ├── srv/
│   │   │   ├── CameraSrv.srv           # Camera service
│   │   │   └── ClassifySrv.srv         # Classification service
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   ├── resource/
│   │   └── test/
│   │
│   ├── grasp_perception/               # Object detection and grasp perception
│   │   ├── grasp_perception/
│   │   │   ├── __init__.py
│   │   │   └── obstacle_node.py        # Obstacle detection node
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   ├── resource/
│   │   └── test/
│   │
│   ├── grasp_algorithms/               # Grasp planning algorithms
│   │   ├── grasp_algorithms/
│   │   │   ├── __init__.py
│   │   │   └── antipodal_node.py       # Antipodal grasp planner
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   ├── resource/
│   │   └── test/
│   │
│   ├── grasp_executor/                 # Grasp execution coordinator
│   │   ├── grasp_executor/
│   │   │   ├── __init__.py
│   │   │   └── panda_grasp_executor.py # Main executor node
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   ├── resource/
│   │   └── test/
│   │
│   ├── grasp_bringup/                  # Grasping system launcher
│   │   ├── grasp_bringup/
│   │   │   ├── __init__.py
│   │   │   └── launch/
│   │   │       └── grasp_pipeline.launch.py # Main grasping pipeline
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   ├── resource/
│   │   └── test/
│   │
│   ├── grasp_interfaces/               # Grasp message definitions
│   │   ├── grasp_interfaces/
│   │   │   ├── __init__.py
│   │   ├── msg/
│   │   │   ├── GraspCandidate.msg      # Single grasp candidate
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   ├── resource/
│   │   └── test/
│   ├── my_node/      # Controls what areas to search and when to grasp.
│   │      ├── my_node/
│   │   ├── resource/
│   │   └── test/
│   └── Beckhoff High Level Controller/  # Beckhoff PLC integration
│       └── Beckhoff_Controller.py                 # PLC communication interface
        └── SuctionValveControl.py   
│
├── install/                            # Built packages (auto-generated)
│   ├── camera_interfaces/
│   ├── camera_module/
│   ├── franka_bringup/
│   ├── franka_control2/
│   ├── franka_description/
│   ├── franka_example_controllers/
│   ├── franka_gripper/
│   ├── franka_hardware/
│   ├── franka_moveit_config/
│   ├── franka_msgs/
│   ├── franka_robot_state_broadcaster/
│   ├── franka_semantic_components/
│   ├── grasp_algorithms/
│   ├── grasp_bringup/
│   ├── grasp_executor/
│   ├── grasp_interfaces/
│   └── grasp_perception/
│
├── build/                              # Build artifacts (auto-generated)
├── log/                                # Build logs (auto-generated)
├── .gitignore                          # Git ignore rules
└── README.md                           # This file
```

---

## Features

### Core Robot Control
- **Motion Planning**: Full MoveIt2 integration for trajectory planning and execution
- **Dual Arm Support**: Single and dual Panda robot configurations
- **Hardware Control**: Real robot control via Franka Control Interface (FCI)
- **Gripper Control**: 2-finger parallel gripper with force/position control
- **Example Controllers**: Joint position, velocity, impedance, and cartesian controllers
- **RViz Visualization**: Complete visualization setup for planning and monitoring

### Autonomous Grasping Pipeline
- **Object Perception**: Vision-based object detection and localization
- **Grasp Planning**: Antipodal grasp algorithm implementation
- **Grasp Execution**: Coordinated arm and gripper control for grasping
- **Force Control**: Force feedback during grasp execution
- **Gripper State Publishing**: Real-time gripper feedback

### Vision and Perception
- **Camera Integration**: Real-time camera input and processing
- **Object Detection**: CNN-based object detection and classification
- **Live Inference**: Real-time model inference on camera streams
- **Camera Calibration**: Support for calibrated camera models
- **Classification Service**: ROS service for object classification

### Integration Capabilities
- **ROS 2 Native**: Full ROS 2 service/action/topic interfaces
- **Modular Architecture**: Independent perception, planning, and execution modules
- **Configuration Driven**: YAML-based parameter configuration
- **PLC Integration**: Beckhoff industrial controller support

---

## Prerequisites

### System Requirements
- **OS**: Ubuntu 22.04 LTS
- **ROS 2**: Humble Hawksbill
- **Python**: 3.10+
- **C++**: 17+ (for franka_arm_ros2 components)
- **Kernel**: Real-time kernel recommended for hardware operation

### Required ROS 2 Packages
```bash
# ROS 2 Humble base installation
sudo apt update
sudo apt install ros-humble-desktop

# Franka-related packages (if not using local franka_arm_ros2)
sudo apt install ros-humble-libfranka

# MoveIt2
sudo apt install ros-humble-moveit ros-humble-moveit-visual-tools

# Gazebo
sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-plugins

# Vision and perception
sudo apt install ros-humble-cv-bridge ros-humble-image-transport
sudo apt install ros-humble-perception-pcl ros-humble-pcl-ros

# Control and interfaces
sudo apt install ros-humble-xacro ros-humble-joint-trajectory-controller
sudo apt install ros-humble-controller-manager ros-humble-hardware-interface
sudo apt install ros-humble-rqt-common-plugins

# Additional tools
sudo apt install build-essential cmake git
```

### Python Dependencies
```bash
# Install Python packages for vision and ML
pip install opencv-python numpy scipy scikit-learn
pip install torch torchvision torchaudio
pip install pillow pyyaml
```

### Hardware Requirements (for real robot operation)
- Franka Control Interface (FCI) enabled on robot
- Robot IP address accessible from your network (default: 172.16.0.2)
- Direct Ethernet connection (dedicated network interface recommended)
- Franka software installed and configured on control station
- Camera with ROS 2 driver support (USB or IP-based)

---

## Setup Instructions

### 1. Clone Repository
```bash
cd ~
git clone https://github.com/N-Raghav/Franka_ROS2_Grasping.git RBE595
cd RBE595
```

### 2. Install Dependencies
```bash
# Install rosdep dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build Workspace
```bash
# Build all packages
colcon build

# Or build specific packages
colcon build --packages-select franka_arm_ros2 camera_module grasp_perception grasp_algorithms grasp_executor grasp_bringup

# For development with symlink install
colcon build --symlink-install
```

### 4. Source Environment
```bash
source install/setup.bash

# Add to ~/.bashrc for automatic sourcing
echo "source ~/RBE595/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 5. Verify Installation
```bash
# Check if packages are discoverable
ros2 pkg list | grep -E "(franka|grasp|camera)"

# Expected output should include franka_arm_ros2 packages, grasp packages, and camera modules
```

---

## Usage

### Simulation (Gazebo + MoveIt)

#### Launch Single Arm with MoveIt
```bash
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=172.16.0.2 use_fake_hardware:=true load_gripper:=true
```

#### Launch Visualization Only
```bash
ros2 launch franka_description visualize_franka.launch.py
```

---

### Hardware Operation

#### 1. Verify Hardware Connection
```bash
# Check network connectivity to robot
ping 172.16.0.2

# Check for Franka errors
ros2 service call /franka_error_recovery_service_server trigger_error_recovery
```

#### 2. Launch Single Arm Hardware Control
```bash
ros2 launch franka_moveit_config moveit_real_arm_platform.launch.py \
  robot_ip:=172.16.0.2 \
  use_fake_hardware:=false
```
#### 3. Launch Example Controller
```bash
# Joint impedance control
ros2 launch franka_bringup joint_impedance_example_controller.launch.py \
  robot_ip:=172.16.0.2

# Gravity compensation
ros2 launch franka_bringup gravity_compensation_example_controller.launch.py \
  robot_ip:=172.16.0.2
```

---

### Grasping Pipeline

#### Launch Full Grasping System
```bash
ros2 launch grasp_bringup grasp_pipeline.launch.py
```

#### Camera and Perception
```bash
# Launch camera node
ros2 run camera_module camera_node

# Run inference on camera stream
ros2 run camera_module infer_live.py

# Calibrate camera
ros2 run camera_module calibrate_camera.py
```

#### Grasp Planning and Execution
```bash
# Run perception (obstacle detection)
ros2 run grasp_perception obstacle_node.py

# Run grasp planning (antipodal algorithm)
ros2 run grasp_algorithms antipodal_node.py

# Run grasp executor
ros2 run grasp_executor panda_grasp_executor.py
```

#### Classification Service
```bash
# Call classification service
ros2 service call /classify_service camera_interfaces/ClassifySrv \
  "image_path: '/path/to/image.jpg'"
```

---

### Monitoring and Debugging

#### View Robot State
```bash
# Monitor joint states
ros2 topic echo /joint_states

# Monitor Franka state
ros2 topic echo /franka_robot_state

# Monitor gripper state
ros2 topic echo /gripper_action_server/feedback
```

#### View Grasping Topics
```bash
# Obstacle detection
ros2 topic echo /obstacles

# Grasp candidates
ros2 topic echo /grasp_candidates

# Executor feedback
ros2 topic echo /grasp_executor/feedback
```

#### Debug Visualization
```bash
# RViz for MoveIt visualization
ros2 launch franka_moveit_config moveit.launch.py

# ROS 2 graph visualization
rqt_graph

# Topic monitoring and plotting
rqt_topic
```

---

## Grasping Pipeline

### Architecture Overview

```
Camera Stream
     ↓
[Camera Module] → Capture and preprocess images
     ↓
[Grasp Perception] → Detect objects and obstacles
     ↓
[Grasp Algorithms] → Generate grasp candidates (Antipodal)
     ↓
[Grasp Planning] → Select best grasp candidate
     ↓
[MoveIt] → Plan arm trajectory to grasp pose
     ↓
[Grasp Executor] → Execute trajectory and gripper control
     ↓
Success/Failure ← Feedback and result
```

### Module Descriptions

#### camera_module
- **Purpose**: Real-time camera input, preprocessing, and inference
- **Key Files**: 
  - `camera.py`: Camera interface abstraction
  - `camera_node.py`: ROS 2 node for camera streams
  - `infer.py`: Offline inference pipeline
  - `infer_live.py`: Real-time inference on live camera
  - `classify_server.py`: Classification service for object detection
- **Outputs**: Image streams, detection results, classified objects

#### grasp_perception
- **Purpose**: Object detection and obstacle identification
- **Key Files**:
  - `obstacle_node.py`: ROS 2 node for obstacle detection
- **Inputs**: Camera images, point cloud data
- **Outputs**: Detected objects, obstacle maps

#### grasp_algorithms
- **Purpose**: Grasp planning and candidate generation
- **Key Files**:
  - `antipodal_node.py`: Antipodal grasp algorithm implementation
- **Inputs**: Object point clouds, poses
- **Outputs**: Grasp candidate array with quality metrics

#### grasp_executor
- **Purpose**: Grasp execution coordination
- **Key Files**:
  - `panda_grasp_executor.py`: Main executor node
- **Inputs**: Selected grasp, robot state, current joints
- **Outputs**: Execution commands, gripper feedback, success/failure

#### grasp_bringup
- **Purpose**: System launch and integration
- **Key Files**:
  - `grasp_pipeline.launch.py`: Main grasping pipeline launcher
- **Functionality**: Orchestrates perception, planning, and execution modules

---

## Project Components

### franka_arm_ros2 (Official Franka ROS 2)
**Purpose**: Complete Franka Panda robot integration for ROS 2

**Sub-packages**:
- `franka_control2`: Custom control node implementation
- `franka_description`: Robot URDF and visualization
- `franka_moveit_config`: MoveIt2 configuration for motion planning
- `franka_hardware`: Low-level hardware interface
- `franka_gripper`: Gripper action server and control
- `franka_bringup`: Launch files and configurations
- `franka_msgs`: Custom message and action definitions
- `franka_robot_state_broadcaster`: Real-time state publishing

**Key Features**:
- Single and dual arm support
- Real hardware and simulated operation
- Multiple example controllers (impedance, velocity, position)
- Force/torque control capabilities
- Error recovery mechanisms

---

### camera_module
**Purpose**: Vision processing and ML inference

**Key Files**:
- `camera.py`: Abstract camera interface
- `camera_node.py`: ROS 2 node publishing camera streams
- `infer.py`: Model inference pipeline
- `classify_server.py`: ROS service for classification
- `calibrate_camera.py`: Camera calibration utilities
- `train.py`: Training utilities for custom models

**ROS Interfaces**:
- **Topic**: `/camera/image_raw` (sensor_msgs/Image)
- **Service**: `/classify_service` (camera_interfaces/ClassifySrv)

---

### grasp_perception
**Purpose**: Vision-based object and grasp detection

**Key Files**:
- `obstacle_node.py`: Obstacle detection ROS node

**ROS Interfaces**:
- **Topic**: `/obstacles` (detected obstacles)
- **Input**: Camera images, point clouds

---

### grasp_algorithms
**Purpose**: Grasp planning and synthesis

**Key Files**:
- `antipodal_node.py`: Antipodal grasp planner

**ROS Interfaces**:
- **Topic**: `/grasp_candidates` (grasp_interfaces/GraspCandidateArray)

**Algorithm**: 
- Antipodal point pair detection
- Force closure checking
- Grasp quality metrics

---

### grasp_executor
**Purpose**: Grasp execution and control

**Key Files**:
- `panda_grasp_executor.py`: Main executor coordinating arm and gripper

**ROS Interfaces**:
- **Topics**: 
  - `/grasp_executor/feedback` (execution status)
  - `/gripper_action_server/goal` (gripper commands)
  - `/joint_trajectory_controller/follow_joint_trajectory` (arm trajectory)

---

### grasp_bringup
**Purpose**: System integration and launching

**Launch Files**:
- `grasp_pipeline.launch.py`: Main pipeline with all modules

**Orchestrates**:
- Camera module
- Perception module
- Planning algorithms
- Execution module
- MoveIt integration

---

### Beckhoff High Level Controller
**Purpose**: Integration with Beckhoff PLC systems

**File**: `beckhoff.py` - Communication interface with industrial controllers

---

## Troubleshooting

### Common Errors and Solutions

#### 1. **Build Failures - Missing Dependencies**
**Error**: `Could not find a package configuration file`

**Solution**:
```bash
# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Try building specific packages
colcon build --packages-select franka_arm_ros2
```

---

#### 2. **URDF/Xacro Parsing Errors**
**Error**: `XML parsing error` or `Failed to parse robot description`

**Solution**:
```bash
# Verify URDF expansion
ros2 run xacro xacro ~/RBE595/src/franka_arm_ros2/franka_description/robots/panda.urdf.xacro > /tmp/panda.urdf

# Check for XML errors
xmllint /tmp/panda.urdf
```

---

#### 3. **MoveIt Launch Failures**
**Error**: `Expected 'value' to be one of [<class 'float'>, ...]`

**Solution**:
```bash
# Check for invalid YAML values in config files
# Look for empty values or tuples like '()'
grep -r ":\s*()$" ~/RBE595/src/franka_arm_ros2/franka_moveit_config/config/

# Verify YAML syntax
python3 -c "import yaml; yaml.safe_load(open('config/file.yaml'))"
```

---

#### 4. **Camera Connection Issues**
**Error**: `Could not open camera device`

**Solution**:
```bash
# Check available cameras
v4l2-ctl --list-devices
ls -la /dev/video*

# Fix camera permissions
sudo usermod -a -G video $USER
newgrp video

# Test camera
ros2 run camera_module camera_node
```

---

#### 5. **Hardware Connection Issues**
**Error**: `Cannot connect to robot` or `FCI connection failed`

**Solution**:
```bash
# Verify IP connectivity
ping 172.16.0.2

# Check Franka control station is running
# Check robot is in "Commanding Mode" on pendant

# Test with fake hardware first
ros2 launch franka_moveit_config moveit.launch.py \
  robot_ip:=172.16.0.2 \
  use_fake_hardware:=true
```

---

#### 6. **Gripper Control Issues**
**Error**: `Gripper action server not responding`

**Solution**:
```bash
# Launch gripper
ros2 launch franka_gripper gripper.launch.py

# Verify gripper service
ros2 service list | grep gripper

# Test gripper
ros2 action send_goal /panda_gripper/gripper_action \
  control_msgs/GripperCommand "command: {position: 0.04, max_effort: 200}"
```

---

### Debug Commands

```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# Echo a topic
ros2 topic echo /joint_states

# List all services
ros2 service list

# View node info
ros2 node info /franka_robot_state_broadcaster

# View parameter server
ros2 param list
ros2 param get /franka_control2_node use_fake_hardware

# Record bag file
ros2 bag record -a -o my_rosbag

# RViz visualization
ros2 launch franka_description visualize_franka.launch.py

# ROS 2 graph
rqt_graph
```



### Testing
```bash
# Run package tests
colcon test --packages-select package_name

# Run specific test
python3 -m pytest src/grasp_algorithms/test/test_antipodal.py
```


## References
- [Franka Documentation](https://frankaemika.github.io/)
- [libfranka](https://github.com/frankarobotics/libfranka)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [MoveIt2 Documentation](https://moveit.ros.org/)
- [LCAS Franka Panda ROS2](https://github.com/LCAS/franka_arm_ros2/tree/humble)

---

**Last Updated**: rn 
**Maintainer**: David Li contact at (woshilixinde@gmail.com)
**Repository**: https://github.com/Akataria27/Recycling_MQP.git
