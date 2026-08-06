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
            package='rover2_drill',
            executable='drill_cap_control',
            name='drill_cap_control',
            **config
        ),
        Node(
            package='rover2_drill',
            executable='drill_control',
            name='drill_control',
            **config
        ),
        Node(
            package='rover2_drill',
            executable='linear_actuator_control',
            name='linear_actuator_control',
            **config
        )
    ])
    
