from setuptools import find_packages, setup

package_name = "spectrometry_mechanical"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aaron",
    maintainer_email="aaron1.goyal@gmail.com",
    description="Watchdog-protected science mechanism CAN bridge",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "spectrometry_mechanical = "
            "spectrometry_mechanical.spectrometery_mechanical:main",
        ],
    },
)
