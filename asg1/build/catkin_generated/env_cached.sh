#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/nishome/rlynch/ros/rosbuild_ws/class-code/asg1/build/catkin_generated', type 'exit' to leave"
  . "/nishome/rlynch/ros/rosbuild_ws/class-code/asg1/build/catkin_generated/setup_cached.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/nishome/rlynch/ros/rosbuild_ws/class-code/asg1/build/catkin_generated'"
else
  . "/nishome/rlynch/ros/rosbuild_ws/class-code/asg1/build/catkin_generated/setup_cached.sh"
  exec "$@"
fi
