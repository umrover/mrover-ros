#!/usr/bin/env python3

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(packages=["localization", "util", "navigation", "esw", "teleop"], package_dir={"": "src"})

setup(**d)
