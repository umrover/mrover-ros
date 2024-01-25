#!/usr/bin/env python3

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        "localization",
        "util",
        "util.state_lib",
        "navigation",
        "navigation.failure_identification",
        "esw",
        "teleoperation.teleoperation",
    ],
    package_dir={"": "src"},
)

setup(**d)
