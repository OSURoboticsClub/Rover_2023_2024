from setuptools import find_packages, setup

package_name = 'rover2_spectrometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aaron',
    maintainer_email='aaron@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
            'rclpy',
            'image_transport_py',
        ],
    },
    entry_points={
        'console_scripts': [
            'spectrometry_publisher = rover2_spectrometry.spectrometry_publisher:main'

        ],
    },
)
