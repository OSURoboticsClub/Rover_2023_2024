# Oregon State University Mars Rover Team 2023-2024
## Groundstation Setup

### Getting Started With Groundstation Code
This repo does not contain groundstation code. To view groundstation code and setup, see https://github.com/OSURoboticsClub/Rover-Unity

## Rover Setup

### Getting Started With Rover Code

We integrate 4 software environments to help manage our rover. These include:

ROS2 Humble - https://docs.ros.org/en/humble/index.html

Moveit2 - https://moveit.picknik.ai/main/index.html

Intel Realsense - https://github.com/IntelRealSense/librealsense


#### ROS2 and Moveit
ROS2 serves as our middleware communications manager between subsystems on the rover and between rover and groundstation. ROS2 also serves as a platform which Moveit2 operates on, acting as our IK and path planning solver and executioner for our 6-DOF arm. 

#### Intel Realsense
The realsense SDK provides libraries that help us interact with the Intel Realsense line of cameras. We use these on the rover to perform pointcloud construction and objection avoidance on the Rover. We also have one mounted on the gripper to allow for collision avoidance and planning for the arm.

### Miscellaneous
This code utilizes NVENC h.265 hardware acceleration for our cameras, powered by the Jetson line of Nvidia computers. This is the only hardware specific factor this code requires to run properly. Can also be fixed in appropriate files by changing from NVENC to standard H265ENC gstreamer functions.

Overall instillation and setup is very straight forward, just make sure to follow all instructional guides closely. 




