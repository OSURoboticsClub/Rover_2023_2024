from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'rover2_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'calibration', 'camera_left_chassis'),
            glob('calibration/camera_left_chassis/*')),

        (os.path.join('share', package_name, 'calibration', 'camera_right_chassis'),
            glob('calibration/camera_right_chassis/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='makemorerobot',
    maintainer_email='makemorerobot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'camera_capture = rover2_camera.camera_capture:main',
        'camera_muxing = rover2_camera.camera_muxing:main',
        'camera_ros2_conversion = rover2_camera.camera_ros2_conversion:main',
        ],
    },
)
