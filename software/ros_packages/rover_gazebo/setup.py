from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'rover_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'rover2_control'), glob('rover2_control/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'worlds/materials/textures'), glob('worlds/materials/textures/*.png')),
        (os.path.join('share', package_name, 'worlds/meshes'), glob('worlds/meshes/*.dae')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jn2',
    maintainer_email='jar3dnorth51@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "gps_node = rover_gazebo.GPSNode:main",
            "imu_node = rover_gazebo.imu:main",
        ],
    },
)
