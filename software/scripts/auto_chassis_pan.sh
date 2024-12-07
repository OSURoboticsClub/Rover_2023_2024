ros2 topic pub chassis/pan_tilt/control rover2_control_interface/msg/TowerPanTiltControlMessage "{should_center: 1}" --once
ros2 topic pub chassis/pan_tilt/control rover2_control_interface/msg/TowerPanTiltControlMessage "{relative_pan_adjustment: -200}" -r 1 -t 5
ros2 topic pub chassis/pan_tilt/control rover2_control_interface/msg/TowerPanTiltControlMessage "{relative_pan_adjustment: 200}" -r 1 -t 10
ros2 topic pub chassis/pan_tilt/control rover2_control_interface/msg/TowerPanTiltControlMessage "{should_center: 1}" --once

ros2 topic pub tower/pan_tilt/control rover2_control_interface/msg/TowerPanTiltControlMessage "{should_center: 1}" --once
ros2 topic pub tower/pan_tilt/control rover2_control_interface/msg/TowerPanTiltControlMessage "{relative_tilt_adjustment: -100}" -r 1 -t 10
ros2 topic pub tower/pan_tilt/control rover2_control_interface/msg/TowerPanTiltControlMessage "{should_center: 1}" --once