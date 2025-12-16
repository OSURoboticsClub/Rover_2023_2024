from setuptools import setup
from glob import glob
import os

package_name = 'rover2_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'rover2_control'), glob('rover2_control/*')),
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
            'iris_controller = rover2_control.iris_controller:main',
            'drive_control = rover2_control.drive_control:main',
            'drive_coordinator = rover2_control.drive_coordinator:main',
            'tower_pan_tilt_control = rover2_control.tower_pan_tilt_control:main',
            'chassis_pan_tilt_control = rover2_control.chassis_pan_tilt_control:main',
            'effectors_control = rover2_control.effectors_control:main',
            'joint_position_control = rover2_control.joint_position_control:main',
            'auton_controller = rover2_control.auton_controller:main',
            'joy_to_drive = rover2_control.joy_to_drive:main',
            'auton_typing = rover2_control.auton_typing:main',
            'odrive_drive_control = rover2_control.odrive_drive_control:main',
            'drive_can_control = rover2_control.drive_can_control:main',
            'drill_control = rover2_control.drill_control:main',
            'gripper_control = rover2_control.gripper_control:main',
            'monitor_aruco = rover2_control.monitor_aruco:main'
        ],
    },
)
