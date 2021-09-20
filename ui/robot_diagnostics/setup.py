#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['robot_diagnostics'],
    package_dir={'': 'src'},
    scripts=['scripts/robot_diagnostics']
)

setup(**d)
