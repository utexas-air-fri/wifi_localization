#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/nishome/rlynch/ros/rosbuild_ws/class-code/wifi_lookup/build/devel', type 'exit' to leave"
  . "/nishome/rlynch/ros/rosbuild_ws/class-code/wifi_lookup/build/devel/setup.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/nishome/rlynch/ros/rosbuild_ws/class-code/wifi_lookup/build/devel'"
else
  . "/nishome/rlynch/ros/rosbuild_ws/class-code/wifi_lookup/build/devel/setup.sh"
  exec "$@"
fi
