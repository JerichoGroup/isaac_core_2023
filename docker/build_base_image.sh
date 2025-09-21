#!/bin/bash

set -e

COMPOSE_DIR="docker/base_docker"
IMAGE_NAME="isaacsim_2023_ros_humble:core_base"
CONTAINER_NAME="isaacsim_2023_ros_humble_core_base"
LOG_MSG="Isaac Sim App is loaded."

echo "STEP 1: Building the Docker image..."
docker compose -f "$COMPOSE_DIR/docker-compose.yml" build

echo "STEP 2: Starting the container to warm up Isaac Sim..."
docker compose -f "$COMPOSE_DIR/docker-compose.yml" up -d

echo "STEP 3: Waiting for Isaac Sim to finish loading..."
until docker logs "$CONTAINER_NAME" | grep -q "$LOG_MSG"
do
    sleep 5
done

sleep 10
echo "Done! Isaac Sim has loaded. Shutting down container..."
docker exec $CONTAINER_NAME pkill -f runapp.sh
sleep 10

echo "STEP 4: Committing the container as a warm-start image..."
docker commit $CONTAINER_NAME $IMAGE_NAME

echo "STEP 5: Stopping and removing the container..."
docker compose -f "$COMPOSE_DIR/docker-compose.yml" down

echo "Done! Warm-start image '$IMAGE_NAME' is ready."
