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
        ('share/' + package_name + '/launch/', ['launch/' + 'tower_launch.py']),
        ('share/' + package_name + '/launch/', ['launch/' + 'chassis_launch.py'])
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
            'controller = camera_gimbal_controller.gimbal_control:main'
        ],
    },
)
