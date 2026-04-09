# scorpio_navigation2

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build and Test](https://github.com/scorpio-robot/scorpio_navigation2/actions/workflows/build_and_test.yml/badge.svg)](https://github.com/scorpio-robot/scorpio_navigation2/actions/workflows/build_and_test.yml)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)

> [!CAUTION]
> This feature package is under rapid development, and backward compatibility is not considered.


## 1. Overview

## 2. Quick Start

### 2.1 Setup Environment

- [Ubuntu 24.04](https://releases.ubuntu.com/noble/)
- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/index.html)
- Simulation package（Optional）: [scorpio_simulator](https://github.com/scorpio-robot/scorpio_simulator.git)

### 2.2 Create Workspace

```bash
mkdir -p ~/ros_ws
cd ~/ros_ws
```

```bash
git clone --recursive https://github.com/scorpio-robot/scorpio_navigation2.git src/scorpio_navigation2
```

### 2.3 Build

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release
```

### 2.4 Running

- For Loopback simulation (without gazebo simulator)

  ```bash
  ros2 launch scorpio_nav2_bringup scorpio_loopback_sim_launch.py world:=turtlebot3_house
  ```

- For Gazebo simulation (requires simulation package)

  ```bash
  ros2 launch scorpio_simulator bringup_sim_launch.py
  ```

  ```bash
  ros2 launch scorpio_nav2_bringup scorpio_gz_sim_launch.py slam:=False world:=turtlebot3_house
  ```

- For rosbag replaying

  ```bash
  ros2 bag play TODO --clock
  ```

  ```bash
  ros2 launch scorpio_nav2_bringup scorpio_real_launch.py use_sim_time:=True slam:=False world:=TODO
  ```

- For real world deployment

  ```bash
  ros2 launch scorpio_nav2_bringup scorpio_real_launch.py slam:=False world:=TODO
  ```
