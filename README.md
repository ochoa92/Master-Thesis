# Master-Thesis
Compliant Control of the Kinova JACO² robot for surface polishing

# Building the ROS packages
cd /path/to/desired/folder
mkdir -p catkin_ws/src
cd catkin_ws
source /opt/ros/indigo/setup.sh
catkin_init_workspace src
catkin_make
