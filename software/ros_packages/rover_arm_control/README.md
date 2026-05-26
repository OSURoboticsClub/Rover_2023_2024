# rover_arm_control
`rover_arm_control` is a ros2 package to launch nodes to control the robotic arm. This includes nodes both to simplify commands for high level task nodes and the high level task specific nodes. 

## Installation
- Complete the Rover Setup in the main README.


## Build
- To rebuild just the specific rover package
```bash
colcon build --symlink-install --packages-select rover_arm_control
```

## Dependencies
- ROS2 Humble +
- rover2_arm
- rover_arm_interface

## Usage
This package contains a launch file that launches the various nodes contained within for the rover launching or to test specific nodes. 
- Launching the main launch (This launch file is used in the rovers main launch file)
    ```bash
    ros2 launch rover_arm_control rover_arm_control.launch.py
    ```
- Launching pc_extraction.launch.py: This launch only launches nodes from the pc_prossesing packages. 
    ```bash
    ros2 launch rover_arm_control pc_extraction.launch.py
    ```
- Launching pick and place: This is a launch file for a pick and place project for ROB514. This launches all nodes required for pick and place using just the robotic arm. The `pick_and_place` node is held out for manual ros run to quickly update that node for testing and debugging. 
    ```bash
    ros2 launch rover_arm_control pick_and_place.launch.py
    ```
## Nodes
This packages contains python nodes for arm control, manipulation, or perception specific for arm operations. All nodes can be run on their own using:  
    ```bash
        ros2 run rover_arm_control <node_name>
    ```
- `absolute_move`: This node provides a service call to send a pose for the end effector to move to in the rover arm base link frame.
    - Services:
        - name = `absolute_move` type = RelativeMove
- `aruco_detector`: This node is a part of the ROB514 project and is used to detect an ArUco tag and determine the location the x-axis of the tag's frame intersects with the ground plane. The ground plane is defined as the average largest plane the depth cameras sees. The largest plane is given by the pc_prosseing node `plane_fit` and is averaged in this node. 
- `auton_typing`: This node runs a dumb version of autonomous typing. It relies on the manual alignment to the Q key and hosts a library of static offsets from that position. The node hosts an action server to recieve the phrase to be typed. The node uses a comma seperated strings to identify the keyys in the library. See the library hardcoded in this nodes file for the key names (specifically non alphanumeric) and the `split_csv` function to see how they are decoded. The `test_auton_typing` node shows an example of how to send a call to the this nodes action server. 
    - Actions:
        - name = `auton_typeing` type = `AutonTyping`


## Liscense
This package is liscensed under the Apache 2.0 liscense.

## Maintainer
Jared Northrop  
jar3dnorth51@gmail.com

## Authors
Jared Northrop  



