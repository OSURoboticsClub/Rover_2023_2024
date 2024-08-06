ros2 topic pub /mining/control/linear rover2_control_interface/msg/DrillControlMessage "{speed: 65535, direction: 0}" -r 10 -t 30
ros2 topic pub /mining/control/linear rover2_control_interface/msg/DrillControlMessage "{speed: 65535, direction: 1}" -r 10 -t 30
ros2 topic pub /mining/drill/control rover2_control_interface/msg/DrillControlMessage "{speed: 65535, direction: 0}" -r 10 -t 30
ros2 topic pub /mining/control/compartment rover2_control_interface/msg/MiningControlMessage "{compartment: 1}" --once
sleep 1
ros2 topic pub /mining/control/compartment rover2_control_interface/msg/MiningControlMessage "{compartment: 2}" --once
sleep 1
ros2 topic pub /mining/control/compartment rover2_control_interface/msg/MiningControlMessage "{compartment: 3}" --once
sleep 1
ros2 topic pub /mining/control/compartment rover2_control_interface/msg/MiningControlMessage "{compartment: 0}" --once
