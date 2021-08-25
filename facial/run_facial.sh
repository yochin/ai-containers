#!/usr/bin/bash
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /aai4r/install/setup.sh
export PYTHONPATH=/aai4r/retinaface:$PYTHONPATH
ros2 run facial facial_node