#Code for ROB499 HW7 - Detecting bottles ontop of a table using pointcloud/depth camera data 

##To Test/Grade this code, do the following:

1. Unzip the hw6.zip file into a ros2 workspace src directory.

2. Build the code using colcon build

3. Run the bag file provided in class.

4. Launch the provided code using the bartender.launch.py launch file:

> ros2 launch hw7 bartender.launch.py

Rviz2 should launch with the correct configuration file, and all nodes should be running after launch.

Note that by default: 
- the topmost point cloud of raw data, '/astra_ros/devices/default/point_cloud' is invisible.
- The second and third pointclouds, representing the table + bottles and bottles only point clouds are visible.
- The First marker is visible, showing a bounding box defining the calculated tabletop plane.
- The marker array is visible, showing the approximate centroids of the bottles.

To confirm functionality, toggle visibility of the various topics rviz is subscribed to.