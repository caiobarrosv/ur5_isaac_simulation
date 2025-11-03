- [1. Overview](#1-overview)
- [2. Maintainer](#2-maintainer)
- [3. Installation](#3-installation)
  - [3.1. Extension Activation](#31-extension-activation)
  - [3.2. Dependencies](#32-dependencies)
- [4. Run Isaac Sim](#4-run-isaac-sim)
  - [4.1. Running Isaac Sim with UR5 Scene](#41-running-isaac-sim-with-ur5-scene)
  - [4.2. Running ROS2 Controllers](#42-running-ros2-controllers)
  - [4.3. Running Individual Nodes](#43-running-individual-nodes)
  - [4.4. Available ROS2 Topics](#44-available-ros2-topics)
- [5. Build](#5-build)
  - [5.1. Native Build (without Docker)](#51-native-build-without-docker)
  - [5.2. Docker Build](#52-docker-build)
- [6. Debug](#6-debug)
- [7. Troubleshooting](#7-troubleshooting)
- [8. (Optional) Visualize UR5 Frames using the Universal ROS2 Package](#8-optional-visualize-ur5-frames-using-the-universal-ros2-package)


---
<a name="overview"></a>
# 1. Overview

![RVIZ Visualization of UR5 and its frames](images/simulation.png)

This repository contains the environment to run and test algorithms for the UR5 of the Laboratory of Robotics at Federal University of Bahia (UFBA). It is possible to test sensors integrations such as RGB+D cameras, generate point clouds, etc.

⚠️ The main reason to use this repository is to avoid damaging the robot with untested algorithms such as trajectory planning, grasping generators, etc.


**Keywords:** Robot manipulators, UR5 Simulation, LaR UFBA

---
<a name="maintainer"></a>
# 2. Maintainer

**Authors:** Caio Viturino

**Maintainers:** Caio Viturino (engcaiobarros@gmail.com)

The `ur5_isaac_simulation` package has been tested under [ROS] Humble and Ubuntu 22.04. 

| OS | ROS | Isaac Sim | CUDA | Nvidia Driver |
| :---: | :---: | :---: | :---: | :---: |
| Ubuntu 22.04 | ROS2 Humble | 5.1.0 | 12.0+ | 525+

> :warning: This package is compatible with **Isaac Sim 5.1.0**
> Check the minimum hardware requirements to run Isaac Sim on the [official website](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html). It is highly recommended to have at least an RTX 3060 GPU.


The simplest way to install the nvidia driver is to go to "Additional drivers" and choose `"using NVIDIA driver metapackage from nvidia-driver-525 (proprietary)"`. If you have any trouble installing this driver, uninstall the all the nvidia drivers before installing a new one: 
```bash
sudo apt-get remove --purge '^nvidia-.*'
```
---
# 3. Installation

## 3.1. Extension Activation
Add the exts/exts folder to the extension path and activate the extension. Mark the option AUTOLOAD.

![Extension Configuration](images/extension_contact_ext.png)
![Extension Configuration](images/extension_contact.png)
![Extension Configuration](images/extension_contact2.png)


## 3.2. Dependencies
Please install the following before proceeding:

```bash
sudo apt install ros-humble-vision-msgs ros-humble-control-msgs \
  ros-humble-tf-transformations ros-humble-joint-state-publisher \
  ros-humble-xacro
```

First time using ROS2? You'll probably need to install colcon-common-extensions:
```bash
pip install -U colcon-common-extensions
```
---
# 4. Run Isaac Sim

## 4.1. Running Isaac Sim with UR5 Scene

1. **Open Isaac Sim 5.1.0**
   ```bash
   cd ~/isaac-sim-standalone-5.1.0-linux-x86_64
   ./isaac-sim.sh
   ```

2. **Load the UR5 Scene**
   - In Isaac Sim, go to `File → Open`
   - Navigate to: `/path/to/ur5_ws/src/ur5_isaac_simulation/usd/ur5_isaac_sim.usd`
   - The scene will load with the UR5 robot and Robotiq 2F-140 gripper

3. **Enable ROS2 Bridge** (if not already enabled)
   - Go to `Window → Extensions`
   - Search for "ROS2 Bridge"
   - Enable the extension

4. **Start the Simulation**
   - Click the `Play` button in Isaac Sim
   - The simulation will start publishing ROS2 topics

## 4.2. Running ROS2 Controllers

In a new terminal, source your workspace and launch the controllers:

```bash
# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
cd ~/ur5_ws
source install/setup.bash

# Launch UR5 controllers and RVIZ2
ros2 launch ur5_isaac_simulation ur5_isaac_ros2.launch.py
```

This will start:
- UR5 trajectory controller
- Robotiq 2F-140 gripper controller
- RVIZ2 visualization

## 4.3. Running Individual Nodes

If you prefer to run nodes separately:

```bash
# UR5 main controller
ros2 run ur5_isaac_simulation ur5_isaac_ros2

# UR5 trajectory action server
ros2 run ur5_isaac_simulation ur5_controller_server

# Gripper action server
ros2 run ur5_isaac_simulation gripper_controller_server
```

## 4.4. Available ROS2 Topics

The simulation publishes/subscribes to the following topics:

```bash
# Joint states (from Isaac Sim)
/joint_states

# Joint commands (to Isaac Sim)
/isaac_joint_commands

# Contact sensor data
/ur5_contact_publisher

# Action servers
/ur5_controller (FollowJointTrajectory)
/gripper_controller (FollowJointTrajectory)
```

---
# 5. Build

## 5.1. Native Build (without Docker)

1. **Clone this repository**
   ```bash
   cd ~/ur5_ws/src
   git clone <THIS_REPOSITORY_URL>
   ```

2. **Install package dependencies**
   ```bash
   cd ~/ur5_ws
   rosdep install --from-paths src -y --ignore-src --rosdistro humble
   ```

3. **Build the workspace**
   ```bash
   cd ~/ur5_ws
   source /opt/ros/humble/setup.bash
   colcon build --packages-select ur5_isaac_simulation
   ```

4. **Source the workspace**
   ```bash
   source ~/ur5_ws/install/setup.bash
   ```

## 5.2. Docker Build

For a containerized environment with Isaac Sim 5.1.0 and ROS2 Humble pre-installed:

1. **Navigate to docker directory**
   ```bash
   cd ~/ur5_ws/src/ur5_isaac_simulation/docker
   ```

2. **Build and run the container**
   ```bash
   # Build and run in development mode
   ./run_services.sh -b dev

   # Or run simulation service
   ./run_services.sh -b sim
   ```

3. **Inside the container**
   ```bash
   # Workspace is pre-built at /workspace/ur5_ws
   cd /workspace/ur5_ws
   source install/setup.bash

   # Run Isaac Sim
   /home/user/isaacsim/isaac-sim.sh
   ```

For more details, see [docker/README.md](docker/README.md)

---
# 6. Debug

In order to debug the ROS2 nodes, follow the steps:
- Install this [ROS Extension](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros).
- Create a launch file and include all the nodes you want to debug.
- Open the workspace folder in vscode and build with `ctrl+shift+b` using colcon build. If you have the wrong folder opened in vscode, it is going to create the build folders in the wrong location.
- Create a debug configuration in launch.json with the following parameters:
  ```json
    {
      "name": "ROS: Launch",
      "type": "ros",
      "request": "launch",
      "target": "path_to_launch_file"
    },
  ```

---
# 7. Troubleshooting

## Contact Sensor Extension Not Loading

If the `ur5sim.ros2contactpublisher` extension fails to load:

1. **Check extension path** in Isaac Sim:
   - `Window → Extensions`
   - Click the gear icon
   - Add path: `/path/to/ur5_ws/src/ur5_isaac_simulation/exts/exts`

2. **Enable AUTOLOAD** for the extension

3. **Regenerate the database** if you modified the `.ogn` file:
   ```bash
   cd ~/ur5_ws/src/ur5_isaac_simulation/exts/exts/ur5sim.ros2contactpublisher
   ~/isaac-sim-standalone-5.1.0-linux-x86_64/kit/python/bin/python3 -m omni.graph.tools.ogn \
     ur5sim/ros2contactpublisher/ogn/python/nodes/ROS2ContactPublisher.ogn
   ```

## ROS2 Topics Not Publishing

1. **Verify ROS2 Bridge is active**:
   ```bash
   ros2 topic list
   ```
   You should see `/joint_states`, `/isaac_joint_commands`, etc.

2. **Check ROS_DOMAIN_ID matches** between Isaac Sim and your terminal:
   ```bash
   echo $ROS_DOMAIN_ID
   ```

3. **Restart Isaac Sim** and reload the scene

## Gripper Not Moving

1. **Check contact data format**:
   ```bash
   ros2 topic echo /ur5_contact_publisher
   ```
   Should show: `[left_force, right_force, left_contact, right_contact]`

2. **Verify gripper controller is running**:
   ```bash
   ros2 node list | grep gripper
   ```

3. **Check force threshold** in `config/params.yaml` (default: 500N)

## Isaac Sim Performance Issues

- Reduce physics simulation rate
- Disable real-time rendering
- Close other GPU-intensive applications
- See [Isaac Sim Linux Troubleshooting](https://docs.isaacsim.omniverse.nvidia.com/latest/troubleshooting.html)

---
# 8. (Optional) Visualize UR5 Frames using the Universal ROS2 Package

You might want to visualize the UR5 frames in order to study direct or inverse kinematics.
Follow the next steps to visualize the UR5 frames in RVIZ and control it using joint state publisher.

```bash
sudo apt-get install ros-humble-joint-state-publisher-gui
```

Clone the Universal Robots ROS2 Description package into your workspace
```bash
cd YOUR_WORKSPACE/src
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description
```

Checkout the branch to the ROS2 version (ros2 branch). The ros2 branch is the main branch.
Just verify if the branch is the ros2.
```bash
cd YOUR_WORKSPACE/src/Universal_Robots_ROS2_Description
git checkout ros2
```

Install the package dependencies:
```bash
cd YOUR_WORKSPACE
rosdep install --from-paths src -y --ignore-src --rosdistro humble
```

Source ROS2 and build the packages
```bash
cd YOUR_WORKSPACE
source /opt/ros/humble/setup.bash
colcon build
```

Source your workspace
```bash
cd YOUR_WORKSPACE
source install/setup.bash
```

Launch the `view_ur.launch.py` in order to see the UR5 frames.
```bash
ros2 launch ur_description view_ur.launch.py ur_type:=ur5
```

In RVIZ, open the configuration file in the folder `config` to get a better view of the UR5 and its frames:
![RVIZ Visualization of UR5 and its frames](images/ur5_rviz_config.png)