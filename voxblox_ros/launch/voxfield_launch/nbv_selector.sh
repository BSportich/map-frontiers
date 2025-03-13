#!/bin/bash

if (( $# < 4 )); then
    >&2 echo 'Too few arguments, expecting four: <sub_sample_size><timer><verbose><extra_viz>'
    exit 1
fi

# Expecting these two rosparam to be already set
NUM_DRONES=$(rosparam get /number_of_drones)
DRONE_NAME=$(rosparam get /drone_name)

echo "Creating $NUM_DRONES nbv selector"

# Acquiring passed arguments
SUB_SAMPLE=$1
TIMER=$2
VERBOSE=$3
VIZ=$4

# Loop through the number of drones and launch each
for ((i=0; i<NUM_DRONES; i++)); do
    namespace="/${DRONE_NAME}_${i}"
    roslaunch map_frontiers NBV_selector.launch namespace:=$namespace sub_sample_size:=$SUB_SAMPLE timer:=$TIMER verbose:=$VERBOSE extra_viz:=$VIZ &
done

# Wait for all launched processes to complete
wait