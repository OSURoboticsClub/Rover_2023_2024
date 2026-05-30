# Oregon State University Mars Rover Team 2023-2024

## Table of Contents
- [Groundstation Setup](#groundstation-setup)
- [Rover Setup](#rover-setup)
  - [Software Dependencies](#software-dependencies)
  - [ROS2 Humble](#ros2-humble)
  - [MoveIt2](#moveit2)
  - [Intel RealSense](#intel-realsense)
  - [GStreamer](#gstreamer)
- [Building the Project](#building-the-project)
- [Miscellaneous Notes](#miscellaneous-notes)

---

## Groundstation Setup

### Getting Started With Groundstation Code

This repo does not contain groundstation code. To view groundstation code and setup, see the [Rover-Unity repository](https://github.com/OSURoboticsClub/Rover-Unity).

---

## Rover Setup

### Getting Started With Rover Code

We integrate several software environments to help manage our rover. These include:

| Software | Purpose | Link |
|---|---|---|
| ROS2 Humble | Middleware & communications | [docs.ros.org](https://docs.ros.org/en/humble/index.html) |
| MoveIt2 | IK & path planning | [moveit.picknik.ai](https://moveit.picknik.ai/main/index.html) |
| Intel RealSense SDK | Depth cameras & pointcloud | [github.com/IntelRealSense](https://github.com/IntelRealSense/librealsense) |
| GStreamer | Camera streaming & encoding | [gstreamer.freedesktop.org](https://gstreamer.freedesktop.org/) |

---

### Software Dependencies

#### ROS2 Humble

[ROS2 Humble](https://docs.ros.org/en/humble/index.html) is our middleware communications manager between subsystems on the rover and between the rover and groundstation. ROS2 Humble is the current LTS release and targets **Ubuntu 22.04 (Jammy)**.

**Installation:**

Follow the official installation guide for your platform:
- [ROS2 Humble — Ubuntu (Debian packages)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) *(recommended)*
- [ROS2 Humble — Other platforms](https://docs.ros.org/en/humble/Installation.html)

Quick install summary for Ubuntu 22.04:

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble (desktop-full recommended)
sudo apt update && sudo apt upgrade
sudo apt install ros-humble-desktop-full

# Source the environment (add to ~/.bashrc for persistence)
source /opt/ros/humble/setup.bash
```

---

#### MoveIt2

[MoveIt2](https://moveit.picknik.ai/main/index.html) operates on top of ROS2 and serves as our IK and path planning solver for our 6-DOF arm. It handles both planning and execution of arm movements.

**Installation:**

MoveIt2 can be installed via apt after ROS2 Humble is set up:

```bash
sudo apt install ros-humble-moveit
```

For a full setup guide including tutorials, see the [MoveIt2 Getting Started docs](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html).

---

#### Intel RealSense

The [Intel RealSense SDK (librealsense)](https://github.com/IntelRealSense/librealsense) provides libraries for interacting with Intel RealSense depth cameras. We use these to perform pointcloud construction and obstacle avoidance on the rover. An additional RealSense camera is mounted on the gripper for arm collision avoidance and planning.

**Installation:**

Follow Intel's official guide: [Linux Installation — librealsense](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)

```bash
# Register the Intel RealSense repository
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
  | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] \
  https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/librealsense.list

sudo apt update && sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev
```

For the ROS2 RealSense wrapper, also install:

```bash
sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-description
```

---

#### GStreamer

[GStreamer](https://gstreamer.freedesktop.org/) is our multimedia pipeline framework, used for camera streaming and video encoding. This project uses **NVENC H.265 hardware acceleration** powered by Nvidia Jetson hardware.

**Installation:**

```bash
# Core GStreamer runtime and plugins
sudo apt update
sudo apt install \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev

# For Jetson / NVENC hardware-accelerated encoding
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-gl
```

> **Note:** NVENC H.265 (`nvh265enc`) requires an **Nvidia Jetson** device with the appropriate Jetpack SDK and GStreamer Nvidia plugins installed. If you are running on non-Jetson hardware, replace `nvh265enc` with the software encoder `x265enc` in the relevant pipeline files. See [Miscellaneous Notes](#miscellaneous-notes) for details.

Useful resources:
- [GStreamer Documentation](https://gstreamer.freedesktop.org/documentation/)
- [Nvidia Jetson GStreamer Guide](https://developer.nvidia.com/embedded/learn/tutorials/first-picture-csi-usb-camera)
- [GStreamer Plugin Reference](https://gstreamer.freedesktop.org/documentation/plugins_doc.html)

---

### Building the Project

Make sure all dependencies above are installed and your ROS2 environment is sourced before building.

```bash
# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Navigate to your workspace
cd ~/ros2_ws

# Install any missing ROS dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install

# Source the workspace overlay
source install/setup.bash
```

> **Tip:** Add both `source` lines to your `~/.bashrc` so they apply automatically in every terminal session.

---

## Miscellaneous Notes and Stuff

- **NVENC vs. Software Encoding:** This code uses `nvh265enc` (NVENC H.265 hardware acceleration) for camera pipelines, which requires an Nvidia Jetson. To run on other hardware, replace `nvh265enc` with `x265enc` in the relevant GStreamer pipeline definitions.
- Follow all installation guides closely — ROS2 and its ecosystem have strict environment and dependency requirements.
- Ensure your ROS2 workspace is sourced in every terminal session you use for development or testing.
