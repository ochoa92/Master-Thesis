## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['thesis_demo', 'hrl_geom', 'pykdl_utils', 'urdf_parser_py'],
    package_dir={'': 'nodes', 'orocos', 'kinova_api'},
    requires=['std_msgs', 'rospy', 'kinova_msgs', 'geometry_msgs']
)

setup(**setup_args)
