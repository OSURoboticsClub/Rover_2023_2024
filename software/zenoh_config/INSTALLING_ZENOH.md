## Zenoh installation and Configuration for Rover

This assumes that you already have ros installed and have up to date copies of the rover and unity UI code. (Note: to ensure your repos are up to date run `git pull orign gazebo_sim` from both repo directories).

### Step 1: Install Zenoh

To install zenoh run: 

`sudo apt install ros-humble-rmw-zenoh-cpp`.

### Step 2: Configure Zenoh

1. Open your basrc for editing using the command:

    `nano ~/.bashrc`

2. Add the following lines to the end of the file, replacing SRC_DIR with the absolute path of the directory your rover code repo is located in Comment out any other lines which contain `export RMW_IMPLEMENTATION`. 

    ```
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ZENOH_ROUTER_CONFIG_URI='/SRC_DIR/Rover_2023_2024/software/zenoh_config/laptop_other.json5'
    ```

### Step 3: Launch The Zenoh Router

1. If you just installed Zenoh, prep your system by running `source ~/.bashrc` and `ros2 daemon stop`.
2. Start the Zenoh router by running `ros2 run rmw_zenoh_cpp rmw_zenohd`. This node will need to be running before you can use ROS on your system with Zenoh.

### Step 4: Start Zenoh Router as systemd Service (Optional)

1. If you would like to avoid the hassle of starting the Zenoh router every time you use ROS, it is possible to automatically start the Zenoh router when your system boots.

2. First it is necessary to create a shell script which sources ROS, exports necessary environment variables, and starts the Zenoh router. 

3. Create a file named `start_zenoh_router.bash` with the following contents, where SRC_DIR is replaced with the path of your rover code repo as described in section 2. (You can use `nano start_zenoh_router.bash`) to do this.

    ```
    #!/bin/bash

    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ZENOH_ROUTER_CONFIG_URI='/home/makemorerobot/Rover_2023_2024/software/zenoh_config/jetson.json5'

    source /opt/ros/humble/setup.bash

    ros2 daemon stop

    ros2 run rmw_zenoh_cpp rmw_zenohd
    ```

4. Run the following command to create the systemd service definition file:

    `sudo nano /etc/systemd/system/zenoh_router.service`

5. Paste in the following contents and save replacing SCRIPT_PATH with the absolute path to the script you created as part of step 3.

    ```

    ```

6. Run `sudo systemctl enable zenoh_router.service`, to enable the service. Zenoh will now start when you reboot your system.