#ros2 topic pub /cameras/main_navigation/camera_control rover_camera_interface/msg/CameraControlMessage "{enable_large_broadcast: False, enable_small_broadcast: True}" -t 10
#ros2 topic pub /cameras/chassis/camera_control rover_camera_interface/msg/CameraControlMessage "{enable_large_broadcast: False, enable_small_broadcast: True}" -t 10
#ros2 topic pub /cameras/infrared/camera_control rover_camera_interface/msg/CameraControlMessage "{enable_large_broadcast: False, enable_small_broadcast: True}" -t 10
#ros2 topic pub /cameras/gripper/camera_control rover_camera_interface/msg/CameraControlMessage "{enable_large_broadcast: False, enable_small_broadcast: True}" -t 10

ros2 topic pub /cameras/main_navigation/camera_control rover_camera_interface/msg/CameraControlMessage "{enable_large_broadcast: True, enable_small_broadcast: False}" -t 10
ros2 topic pub /cameras/chassis/camera_control rover_camera_interface/msg/CameraControlMessage "{enable_large_broadcast: True, enable_small_broadcast: False}" -t 10
ros2 topic pub /cameras/infrared/camera_control rover_camera_interface/msg/CameraControlMessage "{enable_large_broadcast: True, enable_small_broadcast: False}" -t 10
ros2 topic pub /cameras/gripper/camera_control rover_camera_interface/msg/CameraControlMessage "{enable_large_broadcast: True, enable_small_broadcast: False}" -t 10
