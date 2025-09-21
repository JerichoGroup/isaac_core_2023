#!/bin/bash

set -e

IMAGE_NAME="isaacsim_2023_ros_humble:core_simulation"
CONTAINER_NAME="isaacsim_2023_ros_humble_core_simulation"
COMPOSE_FILE="./docker/simulation_docker/docker-compose.yml"
LOG_MSG="RTX ready"

echo "Building simulation image..."
docker compose -f $COMPOSE_FILE build

echo "Starting simulation container to warm up extensions..."
docker compose -f $COMPOSE_FILE up -d

echo "Waiting for simulation to initialize extensions..."
until docker logs "$CONTAINER_NAME" | grep -q "$LOG_MSG"
do
    sleep 5
done

sleep 10

echo "Gracefully stopping Isaac Sim inside container..."
docker exec $CONTAINER_NAME pkill -f "isaac"
sleep 10

echo "Committing container state to image: $IMAGE_NAME"
docker commit $CONTAINER_NAME $IMAGE_NAME

echo "Cleaning up container..."
docker compose -f $COMPOSE_FILE down
sleep 10

echo "Done! Image committed with initialized extensions. You can now run:"
echo "./run_docker.sh"
