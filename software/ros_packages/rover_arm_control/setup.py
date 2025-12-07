from setuptools import find_packages, setup
from glob import glob
import os


package_name = 'rover_arm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jn2',
    maintainer_email='jar3dnorth51@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pick_and_place = rover_arm_control.pick_and_place:main',
            'relative_move = rover_arm_control.relative_move:main',
            'absolute_move = rover_arm_control.absolute_move:main',
            'gripper_control = rover_arm_control.gripper_control:main',
            'aruco_detector = rover_arm_control.aruco_detector:main',
        ],
    },
)
