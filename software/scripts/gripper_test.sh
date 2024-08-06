ros2 topic pub /gripper/control rover2_control_interface/msg/GripperControlMessage "{target: 500}" -t 10
ros2 topic pub /gripper/control rover2_control_interface/msg/GripperControlMessage "{toggle_light: True}" -t 2
ros2 topic pub /gripper/control rover2_control_interface/msg/GripperControlMessage "{toggle_laser: True}" -t 2
ros2 topic pub /gripper/control rover2_control_interface/msg/GripperControlMessage "{should_home: True}" --once
