#!/usr/bin/bash
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /aai4r/install/setup.sh
ros2 run dummybot_ipcam showimg
