#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['avena_view'],
    package_dir={'': 'src'},
    scripts=['scripts/avena_view']
)

setup(**d)
