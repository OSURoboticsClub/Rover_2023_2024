sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
sudo ip link set can1 down
sudo ip link set can1 up

source /opt/ros/humble/setup.bash
/home/makemorerobot/Rover_2023_2024/software/install/setup.bash
ros2 launch rover2_main rover2_main_launch.py
