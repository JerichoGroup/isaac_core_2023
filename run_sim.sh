#!/usr/bin/bash

USD_PATH="--usd-path $PWD/usd/maps/earth/earth.usda"
COM="--com-udp"
BBOX="--bbox-publisher"
DISTANCE_SENSOR="--distance-sensor"

RUN_SIMULATION="ISAACSIM_PYTHON ./simulation/main_sim.py $USD_PATH $COM"

bash -ic "$RUN_SIMULATION"

PID=$!