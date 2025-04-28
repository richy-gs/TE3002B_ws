import os
from glob import glob

from setuptools import find_packages, setup

package_name = "ros2_bridge_sim"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="roli_005",
    maintainer_email="jesusrg2405@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "image_subscriber = ros2_bridge_sim.image_subscriber:main",
            "teleop_controller = ros2_bridge_sim.teleop_controller:main",
            "controller_with_image = ros2_bridge_sim.controller_with_cam:main",
        ],
    },
)
