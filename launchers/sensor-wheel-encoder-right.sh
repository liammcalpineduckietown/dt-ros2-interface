#!/bin/bash

source /environment.sh

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

WHEEL=right

# TODO: update to ROS2 launch
exec roslaunch --wait wheel_encoder_driver wheel_encoder_driver_node.launch veh:=$VEHICLE_NAME wheel:=$WHEEL


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE
