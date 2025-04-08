#!/bin/bash

DRONE_NAME=$(rosparam get /drone_name)

echo "Launching viz for $DRONE_NAME"

rosrun rviz rviz -d $(rospack find map_frontiers)/rviz/$DRONE_NAME.rviz  &

# Wait for all launched processes to complete
wait