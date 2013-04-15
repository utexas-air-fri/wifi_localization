#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/bwi/ros/rosbuild_ws/ardrone_fly/build/catkin_generated', type 'exit' to leave"
  . "/home/bwi/ros/rosbuild_ws/ardrone_fly/build/catkin_generated/setup_cached.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/bwi/ros/rosbuild_ws/ardrone_fly/build/catkin_generated'"
else
  . "/home/bwi/ros/rosbuild_ws/ardrone_fly/build/catkin_generated/setup_cached.sh"
  exec "$@"
fi
