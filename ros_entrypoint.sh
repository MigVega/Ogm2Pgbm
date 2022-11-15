#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$CATKIN_WS/devel_isolated/setup.bash"
exec "$@"
