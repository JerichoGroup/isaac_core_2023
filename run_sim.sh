#!/usr/bin/bash

USD_PATH="--usd-path $PWD/usd/maps/earth/earth.usda"
COM="--com-udp"

RUN_SIMULATION="ISAACSIM_PYTHON ./simulation/main_sim.py $USD_PATH $COM --distance-sensor"

bash -ic "$RUN_SIMULATION"

PID=$!