# xarm_ros2

For Simplified Chinese version: [简体中文版](./ReadMe_cn.md)

## Table of Contents
1. [Introduction](#1-introduction)
2. [Update History](#2-update-history)
3. [Preparation](#3-preparation)
    - [3.1 Install ROS2](#31-install-ros2)
    - [3.2 Install Moveit2](#32-install-moveit2)
    - [3.3 Install Gazebo](#33-install-gazebo)
    - [3.4 Install gazebo_ros_pkgs](#34-install-gazebo_ros_pkgs)
4. [How To Use](#4-how-to-use)
    - [4.1 Create a workspace](#41-create-a-workspace)
    - [4.2 Obtain source code](#42-obtain-source-code)
    - [4.3 Update repository](#43-update-repository)
    - [4.4 Install dependencies](#44-install-dependencies)
    - [4.5 Build xarm_ros2](#45-build-xarm_ros2)
5. [Package Introduction](#5-package-introduction)
    - [5.1 xarm_description](#51-xarm_description)
    - [5.2 xarm_msgs](#52-xarm_msgs)
    - [5.3 xarm_sdk](#53-xarm_sdk)
    - [5.4 xarm_api](#54-xarm_api)
    - [5.5 xarm_controller](#55-xarm_controller)
    - [5.6 xarm_moveit_config](#56-xarm_moveit_config)
    - [5.7 xarm_planner](#57-xarm_planner)
    - [5.8 xarm_gazebo](#58-xarm_gazebo)
    - [5.9 xarm_moveit_servo](#59-xarm_moveit_servo)
6. [Instruction on Major Launch Arguments](#6-instruction-on-major-launch-arguments)
7. [Instruction on xArm5 Pick and Place Task](#7-instruction-on-xarm5-pick-and-place-task)

---

## 1. Introduction

This repository contains simulation models, and corresponding motion planning and controlling demos of the xArm series from UFACTORY. The development and test environment is as follows:

- Ubuntu 20.04 + ROS Foxy
- Ubuntu 20.04 + ROS Galactic
- Ubuntu 22.04 + ROS Humble
- Ubuntu 22.04 + ROS Rolling

Please switch to the corresponding code branch according to different ROS2 versions:

- Foxy: [foxy](https://github.com/xArm-Developer/xarm_ros2/tree/foxy)
- Galactic: [galactic](https://github.com/xArm-Developer/xarm_ros2/tree/galactic)
- Humble: [humble](https://github.com/xArm-Developer/xarm_ros2/tree/humble)
- Rolling: [rolling](https://github.com/xArm-Developer/xarm_ros2/tree/rolling)

---

## 2. Update History

- Moveit dual arm control (under single RViz GUI), each arm can be separately configured.
- Add support for Gazebo simulation, can be controlled by Moveit.
- Support adding customized tool model.
- (2022-09-07) Change the parameter type of service (`set_tgpio_modbus_timeout`/`getset_tgpio_modbus_data`), add parameters to support transparent transmission.
- (2022-09-07) Change topic name (`xarm_states` to `robot_states`).
- (2022-09-07) Update submodule `xarm-sdk` to version 1.11.0.
- (2022-09-09) [Beta] Support Humble version.
- (2022-10-10) `xarm_api` adds some services.
- (2022-12-15) Add parameter `add_realsense_d435i` to load RealSense D435i camera model and support Gazebo simulation.
- (2023-03-29) Added the launch parameter `model1300`, replaced the model of the end of the xArm robot arm with the 1300 series.
- (2023-04-20) Updated the URDF file to load inertia parameters of the link from the configuration file.
- (2023-06-07) Added support for UFACTORY850 robotic arm.
- (2023-10-12) Added the generation and use of joint kinematics parameter files.
- (2024-01-17) Added support for xarm7_mirror model robotic arm.
- (2024-02-27) Added support for Bio Gripper (parameter `add_bio_gripper`, Lite6 is not supported).
- (2024-04-12) Added `uf_ros_lib` to encapsulate certain functions for calling.

---

## 3. Preparation

### 3.1 Install ROS2

- [Foxy Installation](https://docs.ros.org/en/ros2_documentation/foxy/Installation.html)
- [Galactic Installation](https://docs.ros.org/en/ros2_documentation/galactic/Installation.html)
- [Humble Installation](https://docs.ros.org/en/ros2_documentation/humble/Installation.html)

### 3.2 Install Moveit2

Follow the instructions from the [Moveit2 installation guide](https://moveit.ros.org/install-moveit2/binary/).

### 3.3 Install Gazebo

Follow the instructions from the [Gazebo installation guide](https://classic.gazebosim.org/tutorials?tut=install_ubuntu).

### 3.4 Install gazebo_ros_pkgs

Follow the instructions from the [gazebo_ros_pkgs installation guide](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros).

---

## 4. How To Use

### 4.1 Create a workspace

```bash
cd ~
mkdir -p dev_ws/src
```

### 4.2 Obtain source code

```bash
cd ~/dev_ws/src
# Remember to source ROS2 environment settings first
# Do not omit "--recursive" to include submodules
git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive -b $ROS_DISTRO
```

### 4.3 Update repository

```bash
cd ~/dev_ws/src/xarm_ros2
git pull
git submodule sync
git submodule update --init --remote
```

### 4.4 Install dependencies

```bash
cd ~/dev_ws/src/
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

### 4.5 Build xarm_ros2

```bash
cd ~/dev_ws/
# Remember to source ROS2 and Moveit2 environment settings first
colcon build

# Build selected packages
colcon build --packages-select xarm_api
```

---

## 5. Package Introduction

**Reminder 1:** If multiple people are using ROS2 in the current LAN, to avoid mutual interference, please set `ROS_DOMAIN_ID`.

- [Foxy](https://docs.ros.org/en/ros2_documentation/foxy/Concepts/About-Domain-ID.html)
- [Galactic](https://docs.ros.org/en/ros2_documentation/galactic/Concepts/About-Domain-ID.html)
- [Humble](https://docs.ros.org/en/ros2_documentation/humble/Concepts/About-Domain-ID.html)

**Reminder 2:** Remember to source the environment setup script before running any applications in xarm_ros2.

```bash
cd ~/dev_ws/
source install/setup.bash
```

**Reminder 3:** All following instructions will be based on xArm6. Please use proper parameters or filenames for xArm5 or xArm7.

**Reminder 4:** The `<hw_ns>` described below should be replaced with the actual one. For xArm series, the default is `xarm`, and for others, the default is `ufactory`.

### 5.1 xarm_description

This package contains robot description files and 3D models of xArm. Models can be displayed in RViz by the following command:

```bash
ros2 launch xarm_description xarm6_rviz_display.launch.py [add_gripper:=true] [add_vacuum_gripper:=true]
```

Set `add_gripper:=true` to attach xArm gripper model.

### 5.2 xarm_msgs

This package contains all interface definitions for `xarm_ros2`. Please check the instructions in the files before using them. [README](./xarm_msgs/ReadMe.md)

### 5.3 xarm_sdk

The `xarm_sdk` package serves as a submodule of this project, providing SDKs for interfacing with xArm. Refer to the [xArm-CPLUS-SDK](https://github.com/xArm-Developer/xArm-CPLUS-SDK.git) documentation if interested.

### 5.4 xarm_api

This package is a ROS wrapper of `xarm_sdk`. It provides ROS services and topics for communication with real xArms. All services and topics are under `<hw_ns>/` namespace, e.g., the full name for `joint_states` is actually `<hw_ns>/joint_states`.

- **Services:** The services provided correspond to functions in the SDK. Activation of a service depends on the configuration under the `services` domain in `xarm_api/config/xarm_params.yaml` and `xarm_api/config/xarm_user_params.yaml`. Customize parameters by creating `xarm_api/config/xarm_user_params.yaml`.

- **Topics:**
  - `joint_states`: of type `sensor_msgs::msg::JointState`
  - `robot_states`: of type `xarm_msgs::msg::RobotMsg`
  - `xarm_cgpio_states`: of type `xarm_msgs::msg::CIOState`
  - `uf_ftsensor_raw_states`: of type `geometry_msgs::msg::WrenchStamped`
  - `uf_ftsensor_ext_states`: of type `geometry_msgs::msg::WrenchStamped`

**Note:** Some topics are only available when specific `report_type` is set at launch stage.

- **Launch and Test (xArm):**

  ```bash
  # Launch xarm_driver_node
  ros2 launch xarm_api xarm6_driver.launch.py robot_ip:=192.168.1.117

  # Service test
  ros2 run xarm_api test_xarm_ros_client

  # Topic test
  ros2 run xarm_api test_robot_states
  ```

- **Use Command Line (xArm):**

  ```bash
  # Launch xarm_driver_node
  ros2 launch xarm_api xarm6_driver.launch.py robot_ip:=192.168.1.117

  # Enable all joints
  ros2 service call /xarm/motion_enable xarm_msgs/srv/SetInt16ById "{id: 8, data: 1}"

  # Set proper mode (0) and state (0)
  ros2 service call /xarm/set_mode xarm_msgs/srv/SetInt16 "{data: 0}"
  ros2 service call /xarm/set_state xarm_msgs/srv/SetInt16 "{data: 0}"

  # Cartesian linear motion (unit: mm, rad)
  ros2 service call /xarm/set_position xarm_msgs/srv/MoveCartesian "{pose: [300, 0, 250, 3.14, 0, 0], speed: 50, acc: 500, mvtime: 0}"

  # Joint motion for xArm6 (unit: rad)
  ros2 service call /xarm/set_servo_angle xarm_msgs/srv/MoveJoint "{angles: [-0.58, 0, 0, 0, 0, 0], speed: 0.35, acc: 10, mvtime: 0}"
  ```

**Note:** Please study the meanings of [Mode](https://github.com/xArm-Developer/xarm_ros#6-mode-change), State, and available motion instructions before testing on the real robot. Please note the services provided by xArm series and Lite6 have different namespaces.

### 5.5 xarm_controller

This package defines the hardware interface for real xArm control under ROS2.

```bash
# For xArm (xarm6 as example): set 'add_gripper=true' to attach xArm gripper model
ros2 launch xarm_controller xarm6_control_rviz_display.launch.py robot_ip:=192.168.1.117 [add_gripper:=true]

# For Lite6: set 'add_gripper=true' to attach Lite6 gripper model
ros2 launch xarm_controller lite6_control_rviz_display.launch.py robot_ip:=192.168.1.161 [add_gripper:=true]
```

### 5.6 xarm_moveit_config

This package provides capabilities for controlling xArm/Lite6 (simulated or real arm) using Moveit.

- **Simulated:** Launch Moveit, controlling robot in RViz.

  ```bash
  # For xArm (xarm6 as example): set 'add_gripper=true' to attach xArm gripper model
  ros2 launch xarm_moveit_config xarm6_moveit_fake.launch.py [add_gripper:=true]

  # For Lite6: set 'add_gripper=true' to attach Lite6 gripper model
  ros2 launch xarm_moveit_config lite6_moveit_fake.launch.py [add_gripper:=true]
  ```

- **Real Arm:** Launch Moveit, controlling robot in RViz.

  ```bash
  # For xArm (xarm6 as example)
  ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py robot_ip:=192.168.1.117 [add_gripper:=true]

  # For Lite6
  ros2 launch xarm_moveit_config lite6_moveit_realmove.launch.py robot_ip:=192.168.1.161 [add_gripper:=true]
  ```

### 5.7 xarm_planner

This package provides functions for controlling xArm (simulated or real arm) through Moveit API.

```bash
# Simulated xArm: launch xarm_planner_node
ros2 launch xarm_planner xarm6_planner_fake.launch.py [add_gripper:=true]

# Real xArm: launch xarm_planner_node
ros2 launch xarm_planner xarm6_planner_realmove.launch.py robot_ip:=192.168.1.117 [add_gripper:=true]

# In another terminal, run test program (control through API)
ros2 launch xarm_planner test_xarm_planner_api_joint.launch.py dof:=6 robot_type:=xarm
ros2 launch xarm_planner test_xarm_planner_api_pose.launch.py dof:=6 robot_type:=xarm
```

### 5.8 xarm_gazebo

This package supports xArm simulation in Gazebo.

- **Testing xArm on Gazebo independently:**

  ```bash
  # For xArm (xarm6 here)
  ros2 launch xarm_gazebo xarm6_beside_table_gazebo.launch.py

  # For Lite6
  ros2 launch xarm_gazebo lite6_beside_table_gazebo.launch.py
  ```

- **Simulation with Moveit+Gazebo (xArm controlled by Moveit):**

  ```bash
  # For xArm (xarm6 here)
  ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py

  # For Lite6
  ros2 launch xarm_moveit_config lite6_moveit_gazebo.launch.py
  ```

### 5.9 xarm_moveit_servo

This package is a demo for jogging xArm with devices such as joystick, through [moveit_servo](http://moveit2_tutorials.picknik.ai/doc/realtime_servo/realtime_servo_tutorial.html).

- **Controlling with XBOX360 joystick:**

  ```bash
  # For controlling simulated xArm:
  ros2 launch xarm_moveit_servo xarm_moveit_servo_fake.launch.py joystick_type:=1

  # For controlling real xArm:
  ros2 launch xarm_moveit_servo xarm_moveit_servo_realmove.launch.py robot_ip:=192.168.1.123 dof:=5 joystick_type:=1
  ```

- **Controlling with 3Dconnexion SpaceMouse Wireless:**

  ```bash
  # For controlling simulated xArm:
  ros2 launch xarm_moveit_servo xarm_moveit_servo_fake.launch.py joystick_type:=3

  # For controlling real xArm:
  ros2 launch xarm_moveit_servo xarm_moveit_servo_realmove.launch.py robot_ip:=192.168.1.123 dof:=5 joystick_type:=3
  ```

- **Controlling with PC keyboard:**

  ```bash
  # For controlling simulated xArm:
  ros2 launch xarm_moveit_servo xarm_moveit_servo_fake.launch.py dof:=6

  # For controlling real xArm:
  ros2 launch xarm_moveit_servo xarm_moveit_servo_realmove.launch.py robot_ip:=192.168.1.123 dof:=5

  # Then in another terminal, run keyboard input node:
  ros2 run xarm_moveit_servo xarm_keyboard_input
  ```

---

## 6. Instruction on Major Launch Arguments

This section describes various arguments for launching xArm in ROS2.

- **robot_ip**: IP address of xArm, needed when controlling real hardware.

- **report_type** (default: normal): Data report type, supported types are `normal`, `rich`, `dev`.

- **dof** (default: 7): Degree of freedom (DOF) of robot arm. No need to specify explicitly unless necessary.

- **velocity_control** (default: false): Whether to control with velocity interface (otherwise, use position interface).

- **add_realsense_d435i** (default: false): Whether to load the RealSense D435i camera model at the end.

- **add_gripper** (default: false): Whether to include UFACTORY gripper in the model. It has higher priority than `add_vacuum_gripper`.

- **add_bio_gripper** (default: false): Whether to include BIO gripper in the model. `add_gripper` must be false in order to set `add_bio_gripper` to true.

- **add_vacuum_gripper** (default: false): Whether to include UFACTORY vacuum gripper in the model. `add_gripper` must be false in order to set `add_vacuum_gripper` to true.

- **add_other_geometry** (default: false): Whether to add another geometric model as end-tool. `add_gripper` and `add_vacuum_gripper` have to be false.

  - **geometry_type** (default: box): Geometry type to be added as end-tool. Valid types: `box`, `cylinder`, `sphere`, `mesh`.

  - **geometry_mass** (unit: kg, default: 0.1): Model mass.

  - **geometry_height** (unit: m, default: 0.1): Geometry height.

  - **geometry_radius** (unit: m, default: 0.1): Geometry radius.

  - **geometry_length** (unit: m, default: 0.1): Geometry length.

  - **geometry_width** (unit: m, default: 0.1): Geometry width.

  - **geometry_mesh_filename**: Filename of the specified mesh model (place in `xarm_description/meshes/other/`).

  - **geometry_mesh_origin_xyz** (default: "0 0 0")

  - **geometry_mesh_origin_rpy** (default: "0 0 0")

  - **geometry_mesh_tcp_xyz** (default: "0 0 0")

  - **geometry_mesh_tcp_rpy** (default: "0 0 0")

**Example of adding customized end tool (Cylinder):**

```bash
ros2 launch xarm_gazebo xarm6_beside_table_gazebo.launch.py add_other_geometry:=true geometry_type:=cylinder geometry_height:=0.075 geometry_radius:=0.045
```

---

## 7. Instruction on xArm5 Pick and Place Task

- Follow all the above steps for creating the workspace and source in workspace.
- To launch the pick and place task:

  ### In Simulation:

  - Launch the simulation with the argument `add_gripper:=true`:

    ```bash
    ros2 launch xarm_moveit_config xarm5_moveit_fake.launch.py add_gripper:=true
    ```

  - Launch pick and place launch file:

    ```bash
    ros2 launch xarm_planner xarm5_pick_and_place.launch.py
    ```

  ### In Real Robot:

  - Launch the simulation with the arguments `robot_ip:=192.168.1.198` and `add_gripper:=true`:

    ```bash
    ros2 launch xarm_moveit_config xarm5_moveit_realmove.launch.py robot_ip:=192.168.1.198 add_gripper:=true
    ```

  - Launch pick and place launch file:

    ```bash
    ros2 launch xarm_planner xarm5_pick_and_place.launch.py
    ```

---

