from setuptools import find_packages, setup

package_name = 'comms_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #('', ['resource/comms_monitor.cfg'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='osian',
    maintainer_email='leahyo@oregonstate.edu',
    description='Rover Communications Control Package',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            #Node for recording Ubiquiti Status data for logging
            'ubiquiti_logger = comms_control.ubiquiti_logging:main',
            #Generates nodes for monitoring status of various radios
            'comms_monitor = comms_control.comms_monitor:main'
        ],
    },
)
