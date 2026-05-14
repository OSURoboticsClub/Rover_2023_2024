# rover_gazebo
`rover_gazebo` is a ros2 package to launch a gazebo simulation of the rover. 

## Installation
- Clone the git repository to the desired directory.
- Then open the directory and build in the software directory.
- Add the `--continue-on-error` flag to the build command due to issues with other packages in the rover's repository.

```bash
cd "Path/to/Repository"  
cd software
colcon build --continue-on-error 
source install/setup.bash  
```  

## Dependencies
- ROS2 Humble +
- Nav2
- Gazebo Ignition Fortress
- Rover2_arm
- Rover2_control
- Rover_urdf
- Rover_arm_urdf
- odrive_ros2_control

## Usage
This package contains a launch file that launches the gazebo simulation, and teli-op control. 
- Launching the main launch
    ```bash
    ros2 launch rover_gazebo gazebo_drive.launch.py
    ```
- Tele-op Control
    - Tele-op control is achieved through the ROS2 `Joy` node. This has been tested on Xbox and PS controllers where the left sticks vertical movement corresponds to forward and backward while the right sticks horizontal movement correspond to turning left and right. 
    - Tele-op control can be turned on and off through the `stop_teleop_drive` and `start_teleop_drive` service calls. The tele-op drive node sends drive commands at a consistent rate even without user input. This will cause a studdering motion when autonomously driving if left on. Use the start and stop service calls deactivate or activate the tele-op drive node. 

    ```bash
    ros2 service call /stop_teleop_drive std_srvs/srv/Trigger "{}"
    ```
    ```bash
    ros2 service call /start_teleop_drive std_srvs/srv/Trigger "{}"
    ```


## Liscense
This package is liscensed under the Apache 2.0 liscense.

## Maintainer
Jared Northrop  
jar3dnorth51@gmail.com

## Authors
Jared Northrop  



