#!/usr/bin/bash

source /opt/ros/humble/setup.bash
source /root/IsaacSim-ros_workspaces/humble_ws/install/setup.bash;

USD_PATH="--usd-path $PWD/usd/maps/earth/earth.usda"
COM="--com-udp"
BBOX="--bbox-publisher"
DISTANCE_SENSOR="--distance-sensor"
SAT="--sat"

RUN_SIMULATION="/isaac-sim/python.sh ./simulation/main_sim.py $USD_PATH $COM"

bash -ic "$RUN_SIMULATION"

PID=$!
