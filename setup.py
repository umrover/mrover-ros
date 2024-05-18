#!/usr/bin/env python3

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        "localization",
        "perception",
        "util",
        "util.state_lib",
        "navigation",
        "navigation.failure_identification",
        "esw",
        "teleoperation.teleoperation",
        "teleoperation.arm_controller",
        "esw.adafruit_bno08x",
    ],
    package_dir={"": "src"},
)

setup(**d)
