from setuptools import setup
import os
from glob import glob

package_name = "rover2_drill_urc26"

setup(
    name=package_name,
    version="0.0.0",
    py_modules=[
        "linear_actuator_control",
        "linear_actuator_command",
        "drill_control",
        "drill_cap_control",
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aaron",
    maintainer_email="aaron1.goyal@gmail.com",
    description="Drill subsystem nodes for Rover 2",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
            'rclpy'
        ],
    },
    entry_points={
        "console_scripts": [
            "linear_actuator_control = linear_actuator_control:main",
            "linear_actuator_command = linear_actuator_command:main",
            "drill_control = drill_control:main",
            "drill_cap_control = drill_cap_control:main",
        ],
    },
)
