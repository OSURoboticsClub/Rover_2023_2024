from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = {
        'emulate_tty': True,
        'output': 'screen',
        'respawn': True
    }

    return LaunchDescription([
        Node(
            package='rover2_control',
            executable='iris_controller',
            name='iris_controller',
            parameters=[{
                '~port': '/dev/rover/ttyIRIS',
                '~hertz': 20
            }],
            **config
        ),
        Node(
            package='rover2_control',
            executable='drive_control',
            name='rear_bogie',
            parameters=[{
                '~port': '/dev/rover/ttyREAR',
                '~drive_control_topic': 'drive_control/rear',
                '~drive_control_status_topic': 'drive_status/rear',
                '~first_motor_id': 2,
                '~second_motor_id': 1,
                '~invert_second_motor': True
            }],
            **config
        ),
        Node(
            package='rover2_control',
            executable='drive_control',
            name='left_bogie',
            parameters=[{
                '~port': '/dev/rover/ttyLEFT',
                '~drive_control_topic': 'drive_control/left',
                '~drive_control_status_topic': 'drive_status/left',
                '~invert_first_motor': True,
                '~invert_second_motor': True
            }],
            **config
        ),
        Node(
            package='rover2_control',
            executable='drive_control',
            name='right_bogie',
            parameters=[{
                '~port': '/dev/rover/ttyRIGHT',
                '~drive_control_topic': 'drive_control/right',
                '~drive_control_status_topic': 'drive_status/right',
                '~first_motor_id': 2,
                '~second_motor_id': 1
            }],
            **config
        ),
        Node(
            package='rover2_control',
            executable='drive_coordinator',
            name='drive_coordinator',
            **config
        ),
        Node(
            package='rover2_control',
            executable='chassis_pan_tilt_control',
            name='chassis_pan_tilt',
            **config
        ),
        Node(
            package='rover2_control',
            executable='tower_pan_tilt_control',
            name='tower_pan_tilt',
            **config
        ),
        Node(
            package='rover2_control',
            executable='effectors_control',
            name='effectors',
            **config
        ),
        Node(
            package='rover2_control',
            executable='auton_controller',
            name='auton_controller',
            **config
        )
    ])
