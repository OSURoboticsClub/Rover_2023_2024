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
            executable='drive_control',
            name='rear_bogie',
            parameters=[{
                '~port': '/dev/rover/ttyEffectors',
                '~scimech_control_topic_main_actuator': 'scimech_control/main_actuator',
                '~scimech_control_topic_flexinol': 'scimech_control/flexinol',
                '~scimech_control_topic_secondary_actuator': 'scimech_control/secondary_actuator',
                '~drive_control_status_topic': 'drive_status/rear',
                '~first_motor_id': 1,
                '~second_motor_id':2,
		'~third_motor_id':3,
            }],
            **config
        ),


    ])
