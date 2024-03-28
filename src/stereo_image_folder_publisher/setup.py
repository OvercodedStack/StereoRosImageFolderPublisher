#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        'stereo_image_folder_publisher',
    ],
    package_dir={'': 'scripts'}
)


setup(**d)