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
- rover_arm_control_interface
- pc_processing

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
## Notes
The absolute move node uses the rover_arm_gripper as the frame to move, while the relative move uses the rover_arm_tool0. Either one should work, but will cause slight differences on what frame gets to the goal and I believe it is only a small difference between the frames. Note that neither one of them are the gripper fingers. 

Currently 5/26 the uses the ompl (rrtconnect) to plan arm trajectories in this packages. This means they will create paths using a sampled random tree, which can lead to weird and interesting movement choices. It may not take the optimal or shortest path and probably not move in a straight line. 

## Nodes
This packages contains python nodes for arm control, manipulation, or perception specific for arm operations. All nodes can be run on their own using:  
    ```bash
        ros2 run rover_arm_control <node_name>
    ```
- `absolute_move`: This node provides an action server to send a pose for the end effector to move to in the rover arm base link frame.
    - Actions:
        - name = `absolute_move` type = RelativeMove
- `aruco_detector`: This node is a part of the ROB514 project and is used to detect an ArUco tag and determine the location the x-axis of the tag's frame intersects with the ground plane. The ground plane is defined as the average largest plane the depth cameras sees. The largest plane is given by the pc_prosseing node `plane_fit` and is averaged in this node. 
- `auton_typing`: This node runs a dumb version of autonomous typing. It relies on the manual alignment to the Q key and hosts a library of static offsets from that position. Precise manual aligment is needed for the script to work well. The script is hard coded to move 5 cm toward the keyboard to push. The manual aligment needs to be slightly less than 5cm from the keyboard (i.e., the 5-depth required to actuate the key) and completely perpendicular to the keyboard. Differences in angles are far worse than small x or y offsets as they cause large problems for keys further away from Q.  The node hosts an action server to recieve the phrase to be typed. The node uses a comma seperated strings to identify the keys in the library. See the library hardcoded in this nodes file for the key names (specifically non alphanumeric) and the `split_csv` function to see how they are decoded. The `test_auton_typing` node shows an example of how to send a call to the this nodes action server. 
    - Actions:
        - name = `auton_typeing` type = `AutonTyping`
- `move_perp_to_plane`: This node provides a service call to orient the gripper perpendicular to the largest plane seen by the grippers depth camera. The node uses the normal vector that defines the plane as the z-axis (out of palm) of the gripper. The Roll is arbitrarily chosen. Currently the roll leads to the gripper being flipped upside down (i.e., 180 degree rotation about the z-axis from home position). Likely this can be solved by 1. negating the x, 2. negating the x_ref, or 3. switching the order of the cross product when finding x (i.e., x =  np.cross(x_ref, z) -> np.cross(z, x_ref)).
    - Services:
        - name = `move_perp_to_plane` type = `Trigger`
- `relative_move`: This nodes provides a action server to send pose movements relative to the current gripper pose. 
    - Actions:
        - name = `relative_move` type = RelativeMove
- `test_auton_typing`: This node provides a quick way to test the autonomous typing node. The script simply sends an action call to the auton typing node
using a hardcoded string. (Notes to Nolan and Kylan you may just want to use this node for auton typing and add either edit the string during competition setup time or add a parameter where you can type it in when doing ros2 run)


## Liscense
This package is liscensed under the Apache 2.0 liscense.

## Maintainer
Jared Northrop  
jar3dnorth51@gmail.com

## Authors
Jared Northrop  



