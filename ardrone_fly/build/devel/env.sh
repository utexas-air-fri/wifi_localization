#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/bwi/ros/rosbuild_ws/bwi/ardrone_fly/build/devel', type 'exit' to leave"
  . "/home/bwi/ros/rosbuild_ws/bwi/ardrone_fly/build/devel/setup.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/bwi/ros/rosbuild_ws/bwi/ardrone_fly/build/devel'"
else
  . "/home/bwi/ros/rosbuild_ws/bwi/ardrone_fly/build/devel/setup.sh"
  exec "$@"
fi
