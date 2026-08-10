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
                executable='heist_mech_control',
                name='heist_mech_control',
                parameters=[{
                    'can':'can_arm',
                    'can_id':60
                }],
                **config
            )
        ])
