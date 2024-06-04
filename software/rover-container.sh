sudo docker build -t ros2humble -f Dockerfile.roverhumble .
sudo docker run -dit -v /dev:/dev --privileged --net=host --name rover2 ros2humble
