from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    config = {
        'emulate_tty': True,
        'output': 'screen',
        'respawn': True
    }

    return LaunchDescription([
        Node(
            package='rover2_odometry',
            executable='imu',
            name='imu',
            **config
        ),
#        Node(
#            package='rover2_odometry',
#            executable='scimech_sensors',
#            name='scimech_sensors',
#            **config
#        ),

    # odometry and odrive_can_info are for old wheel odom launch
        # Node(
        #     package='rover2_odometry',
        #     executable='odometry',
        #     name='odometry',
        #     **config
        # ),
        Node(
            package='rover2_odometry',
            executable='odrive_can_info',
            name='odrive_can_info',
            **config
        ),

        Node(
            package='rover2_odometry',
            executable='GPSNode',
            name='RTK_GPS',
            **config
        ),

        # Launch visual odom
        #  Node(
        #      package='rtabmap_odom', 
        #      executable='rgbd_odometry', 
        #      output='screen',
        #      parameters=[{
        #          # Frames
        #          'frame_id':'rover_base_origin',
        #          'odom_frame_id': "odom",

        #          # config params
        #          'publish_tf':False, 
        #          'approx_sync':False,
        #          'wait_imu_to_init':False,
in rover_drive.yaml it does odometry. Take that topic instead, possibly, EVALUATE
        #          # Internal Params (must be strings)
        #          'Odom/Strategy':'1',        # 0=Frame-to-Map (F2M) 1=Frame-to-Frame (F2F) 2=Fovis 3=viso2 4=DVO-SLAM 5=ORB_SLAM 6=OKVIS 7=LOAM 8=MSCKF_VIO 9=VINS-Fusion 10=OpenVINS 11=FLOAM 12=Open3D 13=cuVSLAM
        #             # There are paramters to set for each strategy 
        #         #  'Odom/FilteringStrategy':'0',        # 0=No filtering (default), 1 = Kalman , 2 = particle filter. Just for smoothing, not combining
        #             # There are parameters to set for each filter too
        #         #  'Odom/ResetCountdown':'10',             # "Automatically reset odometry after X consecutive images where odometry cannot be computed (a value of 0 disables auto-reset). When a reset occurs, odometry resumes from the last successfully computed pose with large covariance to trigger a new map. If external odometry is used, it will also be reset based on the motion estimated relative to the last computed pose but no large covariance will be received, so that a new map won't be triggered.
        #         #  'Odom/Holonomic':'false',
        #         #  'Odom/GuessSmoothingDelay':'0',       # 0 Default. Estimated velocity is averaged based on last transforms up to this maximum delay. This can help to get smoother velocity prediction. If filtering is set or delay is below odometry rate, this is ignored
        #      }],
        #      remappings=[
        #          ('rgb/image', '/camera/d455/color/image_raw'),
        #          ('rgb/camera_info', '/camera/d455/color/camera_info'),
        #          ('depth/image', '/camera/d455/aligned_depth_to_color/image_raw'),
        #          ('odom', '/odometry/visual'),
        #      ]),

        # Local EKF (odom)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node_odom',
            output='screen',
            parameters=[{
                'print_diagnostics': True,
                'debug': False,
                'frequency': 30.0,
                'two_d_mode': False,
                'publish_tf': True,
                # 'sensor_timeout': 0.1,
                # 'transform_time_offset': 0.0,
                # 'transform_timeout': 0.0,
                
                # 'map_frame': 'map',
                'odom_frame': 'odom',
                'base_link_frame': 'rover_base_origin',
                'world_frame': 'odom',
                
                # Local odometry 
                'odom0': '/drive_controller/odom',                 # KRJ TODO: evaulate if this odom from new drive controller is better or at least equal to our /wheel_odom published from our rover2_odometry odometry node
                'odom0_config': [False, False, False,
                                False, False, False,
                                True,  True,  False,
                                False, False, True,
                                False, False, False],
                'odom0_queue_size': 10,
                'odom0_differential': False,
                'odom0_relative': False,

                # Visual odometry
                # 'odom1': '/odometry/visual',
                # 'odom1_config': [False,  False,  False, # x, y position
                #                 False, False, False,   # yaw orientation
                #                 True, False, False,
                #                 False, False, False,
                #                 False, False, False],
                # 'odom1_queue_size': 10,
                # 'odom1_differential': False,
                # 'odom1_relative': False,
                
                # IMU 
                # (See nav2 gps docs REP 105 odom frame should use only heading from IMU)
                # [false, false, false, false,  false,  true, false, false, false, false,  false,  false, false,  false,  false]
                'imu0': '/imu/data',
                'imu0_config': [False, False, False,
                               True, True, True,
                               False, False, False,
                               False, False, False,
                               False,  False,  False],
                'imu0_queue_size': 10,
                'imu0_differential': False,
                'imu0_relative': False,
                'imu0_remove_gravitational_acceleration': False,
            }],
            remappings=[
                ('/odometry/filtered', '/odometry/local'),
            ]
        ),
        
        # Global EKF (map)
        TimerAction(
         period=1.0,  # Delay in seconds
         actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_node_map',
                output='screen',
                parameters=[{
                    'print_diagnostics': True,
                    'debug': False,
                    'publish_tf': True,
                    'frequency': 10.0,
                    'two_d_mode': True,
                    # 'sensor_timeout': 0.1,
                    # 'transform_time_offset': 0.0,
                    # 'transform_timeout': 0.0,
                    
                    'odom_frame': 'odom',
                    'base_link_frame': 'rover_base_origin',
                    'world_frame': 'map',
                    'map_frame': 'map', # published map frame tf name
                    
                    # Local odometry 
                    'odom0': '/drive_controller/odom',
                    'odom0_config': [False, False, False,
                                    False, False, False,
                                    True,  True,  False,
                                    False, False, True,
                                    False, False, False],
                    'odom0_queue_size': 10,
                    'odom0_differential': False,
                    'odom0_relative': False,

                    # GPS odometry (from navsat_transform) (pose x, y)
                    'odom1': 'odometry/gps',
                    'odom1_config': [True,  True,  False,   # x, y position
                                    False, False, False,
                                    False, False, False,
                                    False, False, False,
                                    False, False, False],
                    'odom1_queue_size': 10,
                    'odom1_differential': False,
                    'odom1_relative': False,
                    
                    # IMU (accel x, vel yaw)
                    'imu0': '/imu/data',
                    'imu0_config': [False, False, False,
                                    True, True, True,
                                    False, False, False,
                                    False, False, False,
                                    False,  False,  False],
                    'imu0_queue_size': 10,
                    'imu0_differential': False,
                    'imu0_relative': False,
                    'imu0_remove_gravitational_acceleration': False,
                }],
                remappings=[
                    ('/odometry/filtered', 'odometry/global'),
                ]
            )]
        ),


        # 5. NavSat Transform - converts GPS to map frame
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[{
                # Frequency and timing
                'frequency': 30.0,
                'delay': 3.0,

                # Magnetic declination at your location (radians)
                # Find yours: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
                # 'magnetic_declination_radians': 0.253,

                # Use odometry heading instead of IMU
                'use_odometry_yaw': False,
                #'yaw_offset': 1.570796326, # yaw correction of IMU absolute yaw measurement (must point east)

                # 2D navigation
                'zero_altitude': True,

                # Publishing options
                'broadcast_cartesian_transform': True,
                'publish_filtered_gps': True,

                # Let first GPS message set origin
                'wait_for_datum': True,
                #'use_manual_datum': True,
                #'datum': [44.56722346625757, -123.27433385957002, 0.0],

                'base_link_frame_id': 'rover_base_origin',
                'world_frame_id': 'map',  # Match the EKFs
            }],
            remappings=[
                ('/imu', '/imu/data'),                  # IMU topic
                ('/gps/fix', '/gps/fix'),                    # GPS INPUT TOPIC
                ('/gps/filtered', '/gps/filtered'),          # GPS INPUT TOPIC
                ('/odometry/filtered', '/odometry/global'),  # Which EKF to use for heading
                ('/odometry/gps', '/odometry/gps'),          # GPS output topic
            ]
        ),
    ])
