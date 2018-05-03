#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['osm_cartography'],
    package_dir={'': 'src'},
    install_requires=['rospy',
                      'geodesy',
                      'geographic_msgs',
                      'geometry_msgs',
                      'std_msgs',
                      'visualization_msgs'],
    )

setup(**d)
