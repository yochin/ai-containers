#!/usr/bin/bash
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /aai4r/install/setup.sh
export PYTHONPATH=$PYTHONPATH:/aai4r/aai4r-ServiceContextUnderstanding
export PYTHONPATH=$PYTHONPATH:/aai4r/aai4r-ServiceContextUnderstanding/lib
ros2 run meal meal_node