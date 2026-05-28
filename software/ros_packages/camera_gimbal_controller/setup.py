from setuptools import find_packages, setup

package_name = 'camera_gimbal_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/', ['launch/' + 'launch_tower.py']),
        ('share/' + package_name + '/launch/', ['launch/' + 'launch_chassis.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nolan',
    maintainer_email='kesslnol@oregonstate.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tower_controller = camera_gimbal_controller.tower_gimbal_control:main',
            'chassis_controller = camera_gimbal_controller.chasssis_gimbal_control:main'
        ],
    },
)
