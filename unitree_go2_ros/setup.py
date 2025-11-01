"""Setup script for unitree_go2_ros package."""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = "unitree_go2_ros"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="your.email@example.com",
    description="ROS 2 control interface for Unitree Go2 robot via SBUS protocol",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "go2_control_node = unitree_go2_ros.go2_control_node:main",
        ],
    },
)
