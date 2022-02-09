#!/bin/bash

if [ $# == 0 ]; then
    echo ">>> USAGE: $0 private_repo_ssh_url"
    echo ">>> private_repo_ssh_url: SSH URL of private FEAGI repo"
    exit 1
else
    export DOCKER_BUILDKIT=1
    export PRIVATE_FEAGI_REPO=$1
    docker build --ssh default \
                 --build-arg REPO=$1 \
                 -f Dockerfile.private . --no-cache
    docker build ../third_party/grafana
    docker build ../third_party/godot/container
    docker-compose -f feagi.yml -f feagi.private.yml up
fi
