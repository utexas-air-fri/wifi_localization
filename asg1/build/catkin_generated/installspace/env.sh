#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/usr/local', type 'exit' to leave"
  . "/usr/local/setup.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/usr/local'"
else
  . "/usr/local/setup.sh"
  exec "$@"
fi
