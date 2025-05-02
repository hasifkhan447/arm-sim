#!/bin/bash 
set -e 

source /opt/ros/humble/setup.bash 
source /usr/share/gazebo-11/setup.sh

if [ -f /home/rosdev/workspace/install/setup.bash ]; then 
	source /home/rosdev/workspace/install/setup.bash
fi

exec "$@"


