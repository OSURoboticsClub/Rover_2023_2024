from setuptools import setup
from glob import glob

package_name = 'rover2_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*launch.[pxy][yma]*'))
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
            'auton_controller = rover2_control.auton_controller:main'
        ],
    },
)
