# DAM ROBOTICS CLUB ROVER ARM CAPSTONE 2024
This code contains the ROS packages for the 2024 rover arm capstone made for the DAM Robotics Club. This was built on ROS2 Humble using the Moveit2! and ros2_control libraries. There is also a modified version of the official odrive_can ros_control plugin included.

To successfully run the code, a socketCAN interface must be running as "can0". Also, a gamepad with valid drivers must be plugged in. 

## Installation
A working installation of ROS2 Humble is required. To install the code, first clone the repository to a new directory:

```
mkdir arm_ws
cd arm_ws
git clone https://www.github.com/PreetDhami/rover_arm_capstone_2024
```
Next, install dependencies using rosdep:

```
sudo apt-get install python3-rosdep
rosdep init
rosdep update
rosdep install --from-paths rover_arm_capstone_2024 -y --ignore-src
```
Then, compile with colcon and source the installation:
```
colcon build && source install/setup.bash
```
Finally, to run the code with gamepad control:
```
ros2 launch rover_arm arm.launch.py
```


## Packages
Below are all the included packages, a brief explanation of their content, and a link to reference documentation for further reading:
- `rover_arm_urdf`: This includes the URDF description of the rover arm, as well as the necessary mesh files (.stl and .dae). This was created by following [this article](https://control.ros.org/humble/doc/ros2_control_demos/example_7/doc/userdoc.html#writing-a-urdf) from `ros2_control` as an example

- `odrive_ros2_control` and `odrive_base`: These were originally made by Odrive Robotics from [this repo](https://github.com/odriverobotics/odrive_can/tree/ros-control), but were modified to consider gear ratios and units when communicating between ROS2 and the Odrive S1's. ROS uses radians, whereas the Odrives use rotations. `ros2_control` uses the angular positions and velocities of each joint rather than those of the motors.

- `joy_to_servo`: The package `moveit_servo` is used to do all calculations for operating the arm from some input. The `joy` package is used to read inputs from a gamepad. This package serves as a bridge between these two. Any efforts to re-map user control to operate the rover should be changed here. As of right now, inverse kinematics does not work with user operation, so all control is done on individual joints. An explanation of this code, as well as `moveit_servo` is found [here](https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html). This code was clone and modified from [this example](https://github.com/ros-planning/moveit2/blob/humble/moveit_ros/moveit_servo/src/teleop_demo/joystick_servo_example.cpp).

- `rover_arm`: Finally, this is the main package which contains the launch files for the arm, as well as ALL of the config files for the other packages.

## Nodes
The main launch file is `arm.launch.py`, however there are other automatically generated launch files that have been left as they are. `demo.launch.py` runs inverse the inverse kinematics solver within RVIZ, so that the user can drag around the end effector and command the arm throught the GUI. 

`arm.launch.py` runs the following nodes:

- `Rviz`: Handles displaying robot to gui. Comment out to disable.

- `control_node`: Loads the rover arm controller and hardware plugin (odrive_ros2_control package) as specified by the rover_arm.urdf.xacro file. Subscribes to `/rover_arm_controller/joint_tragectory` topic, and passes the inputs along to odrive plugin for each joint.

- `robot_state_pub_node`: Publishes the robot state to the `/state` topic. 

- `joint_state_broadcaster_spawner`: spawns the joint_state_broadcaster, which other nodes subscribe to.

- `robot_controller_spawner`: spawns the controller manager to manage multiple controller, if there were any. 

- `joy_node`: Publishes gamepad inputs to `/joy` topic.

- `servo_node`: Listens to `/delta_joint_cmds` and `/delta_twist_cmds`, preforms calculations on whether the requested movement is valid or not (according to kinematics.yaml, rover_arm.urdf.xacro, and servo_config.yaml). `/delta_joint_cmds` Sepcifies how to change the angular positions/velocities of individual joints, whereas `/delta_twist_cmds` should specify how to move the cartesian positions/velocities of robot links (inverse kinematics). Currently, only `/delta_joint_cmds` is functioning. If a requested movement will not have a self-collision and stays within joint limits, then it publishes to the `/rover_arm_controller/joint_tragectory` topic.

- `joy_to_servo_node`: Custom node which listens to `/joy` topic and outputs to `/delta_joint_cmds` and `/delta_twist_cmds`. Change the code of this node if you would like to remap how the gamepad operates the arm.