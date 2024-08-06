ros2 topic pub /tower/light/control rover2_control_interface/msg/LightControlMessage "{light_mode: 1}" --once
ros2 topic pub /tower/light/control rover2_control_interface/msg/LightControlMessage "{light_mode: 2}" --once
ros2 topic pub /tower/light/control rover2_control_interface/msg/LightControlMessage "{light_mode: 0}" --once
ros2 topic echo /tower/status/gps
