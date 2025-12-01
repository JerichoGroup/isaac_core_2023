#!/bin/bash

set -e

COMPOSE_FILE="./docker/simulation_docker/docker-compose.yml"

echo "Stopping the simulation docker container..."
docker compose -f $COMPOSE_FILE down
