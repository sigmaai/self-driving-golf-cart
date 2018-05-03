#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['route_network'],
    package_dir={'': 'src'},
    install_requires=['rospy',
                      'geodesy',
                      'geographic_msgs',
                      'geometry_msgs',
                      'nav_msgs',
                      'rospy',
                      'visualization_msgs'],
    )

setup(**d)
