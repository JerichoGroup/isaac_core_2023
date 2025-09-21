#!/bin/bash

# Source ROS environments
source /opt/ros/humble/setup.bash
source /root/IsaacSim-ros_workspaces/humble_ws/install/setup.bash

export ISAACSIM_PYTHON="/isaac-sim/python.sh"
export ISAACSIM="/isaac-sim/runapp.sh"
export ISAACSIM_PATH="/isaac-sim"
export OMNI_CONFIG_FILE="$ISAACSIM_PATH/apps/omni.isaac.sim.python.kit"

echo -e '\n[dependencies]\n"cesium.omniverse" = {}\n"cesium.usd.plugins" = {}\n"omni.anim.curve" = {}' >> "$OMNI_CONFIG_FILE"
echo -e '\n[settings.app]\nuseFabricSceneDelegate = true' >> "$OMNI_CONFIG_FILE"

# Start Isaac Sim to warm up cache
echo "Starting Isaac Sim for initial cache warm-up..."
/isaac-sim/runapp.sh --no-window
