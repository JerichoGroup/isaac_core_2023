#!/bin/bash

set -e

COMPOSE_FILE="./docker/simulation_docker/docker-compose.yml"

echo "Starting simulation inside docker container..."
docker compose -f $COMPOSE_FILE up -d
