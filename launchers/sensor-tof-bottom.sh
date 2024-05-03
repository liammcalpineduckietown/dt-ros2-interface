#!/bin/bash

source /environment.sh

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# TODO: update to ROS2 launch
exec roslaunch --wait tof_driver tof_driver_node.launch veh:=$VEHICLE_NAME name:=bottom


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE
