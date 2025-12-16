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
            package='rover2_status',
            executable='led_node',
            name='led_node',
            **config
        ),
	Node(
			package='rover2_status',
            executable='node_info',
            arguments=[
              "--ros-args",
              "--disable-stdout-logs"]
        ),
	Node(
			package='rover2_status',
            executable='node_log',
            arguments=[
              "--ros-args",
              "--disable-stdout-logs"]
        ),
	Node(
			package='rover2_status',
            executable='node_topic_detector',
            arguments=[
              "--ros-args",
              "--disable-stdout-logs"]
        ),
	Node(
			package='rover2_status',
            executable='moveit_logger',
            arguments=[
              "--ros-args",
              "--disable-stdout-logs"]
        ),
	Node(
			package='rover2_status',
            executable='drivetrain_telemetry',
        ),
	Node(
			package='rover2_status',
            executable='arm_telem',
        ),

    ])
