sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000 dbitrate 1000000 berr-reporting on fd on
sudo docker start rover2
