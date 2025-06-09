import glob
import os

from setuptools import find_packages, setup

package_name = "ros2_coppeliasim_ws"  # todo en minúscula

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Ahora sí especificamos qué archivos de launch queremos empaquetar:
        (
            os.path.join("share", package_name, "launch"),
            glob.glob("launch/*.py"),
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
            "snake_teleop = ros2_coppeliasim_ws.snake_teleop:main",
            "snake_oscillator = ros2_coppeliasim_ws.snake_oscillator:main",
        ],
    },
)
