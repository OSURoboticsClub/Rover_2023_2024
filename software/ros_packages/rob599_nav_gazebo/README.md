# rob599_nav_gazebo
`rob599_nav_gazebo` is a ros2 package for ROB599 Mobile Robots Timber Tracks Project. This package integrates the Nav2 stack with the mars rover's gazebo sim. This package contains two custom Nav2 costmap plugins: `clearance_layer` and `eleveation_layer`. 

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
- Point Cloud Library
- rover2_arm
- rover2_control
- rover_urdf
- rover_arm_urdf
- odrive_ros2_control
- rover_gazebo

## Usage
This package contains a launch file that launches the gazebo simulation, a Nav2 stack, and teli-op control. 
- Launching the main launch
    ```bash
    ros2 launch rob599_nav_gazebo forest_nav_launch.py
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
- Sending Waypoints
    - Waypoints can be sent through the Nav2 `navigate_to_pose` action server.
    ```bash
     ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: -7.0, y: -5.5, z: 0.0}, orientation: {w: 1.0}}}}"
    ```

## Liscense
This package is liscensed under the Apache 2.0 liscense.

## Maintainer
Jared Northrop  
jar3dnorth51@gmail.com

## Authors
Jared Northrop  
Aswin Arumugam  
Olivia Gehrke  
Rae Moon  



