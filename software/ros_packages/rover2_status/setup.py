from setuptools import setup
from glob import glob

package_name = 'rover2_status'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='hakkilab@oregonstate.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'system_statuses_node = rover2_status.system_statuses_node:main',
            'led_node = rover2_status.led_node:main',
            'drivetrain_telemetry = rover2_status.odrive_telemetry:drivetrain_telem',
            'arm_telem = rover2_status.odrive_telemetry:arm_telem',
            'drive_slip = rover2_status.drive_slip_detection:main',
            'node_topic_detector = rover2_status.node_topic_detector:main',
            'latency = rover2_status.latency:main',
            'node_info = rover2_status.node_info:main',
                        'node_log = rover2_status.node_logs:main',
            'integrator = rover2_status.integrator:main',

            'moveit_logger = rover2_status.moveit_logger:main',

        ],
    },
)
