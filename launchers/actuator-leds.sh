#!/bin/bash

source /environment.sh

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# Initialize launch file
dt-launchfile-init

# Launch the tof_driver node
dt-exec ros2 run led_driver led_driver_node

# Wait for app to end
dt-launchfile-join


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

