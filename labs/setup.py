## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

import sys
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

def read(fname):
    return open(fname).read()

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['image_mean_feature_x', 'flaky_door_opener', 'robot_door_sim'],
    package_dir={'': 'src'},
    platforms=['ROS'],
    long_description="Python nodes for Probabilistic Robotics lab"
)

# overwrite files in dest dir if timestamp later
if 'install' in sys.argv:
    sys.argv.append('--force')

setup(**setup_args)
