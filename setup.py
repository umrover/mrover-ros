#!/usr/bin/env python3

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        "localization",
        "util",
        "navigation",
        "navigation.failure_identification",
        "esw",
        "teleop",
        "teleoperation.teleoperation",
    ],
    package_dir={"": "src"},
)

setup(**d)
