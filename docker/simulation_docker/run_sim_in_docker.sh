#!/usr/bin/bash

alias ISAACSIM_PYTHON="/isaac-sim/python.sh"
alias ISAACSIM="/isaac-sim/python.sh"
alias ISAAC_PATH="/isaac-sim"

source /opt/ros/humble/setup.bash
source /root/IsaacSim-ros_workspaces/humble_ws/install/setup.bash;

TOPIC=$1

if [ "$TOPIC" = "-h" ]; then
    echo "Usage flying_camera.bash topic_odom usd_path"
    exit 0
fi

if [ -z "$TOPIC" ]
    then
        TOPIC=sim/odom
fi
echo $TOPIC

USD_PATH=$2

if [ -z "$USD_PATH" ]
    then
        USD_PATH="$PWD/usd/maps/earth/earth.usda"
fi
echo $USD_PATH

RUN_SIMULATION="$ISAACSIM_PYTHON ./simulation/core_parser.py --usd-path $USD_PATH --odom $TOPIC"

bash -ic "$RUN_SIMULATION"

PID=$!
