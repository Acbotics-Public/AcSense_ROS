#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --

source "/acsense_ros/install/setup.bash" --

exec "$@"
