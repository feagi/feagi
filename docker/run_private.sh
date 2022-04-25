#!/bin/bash

if [ $# -ne 1 ] && [ $# -ne 2 ]; then
    printf "\n\t>>> USAGE: $0 <private_repo_ssh_url> <target_branch>"
    printf "\n\t>>> private_repo_ssh_url: SSH URL of private FEAGI repo"
    printf "\n\t>>> target_branch (optional): name of branch to checkout (default is 'main')\n\n"
    exit 1
else
    ssh-add -A
    export PRIVATE_FEAGI_REPO=$1
    export PRIVATE_REPO_BRANCH=${2:-main}

    docker image rm -f docker_feagi
    DOCKER_BUILDKIT=1 docker build --no-cache --ssh default \
                                   --build-arg REPO=$1 \
                                   --build-arg BRANCH=$PRIVATE_REPO_BRANCH \
                                   -f Dockerfile.private .
    docker-compose -f feagi.yml build --no-cache godot ros-gazebo
    docker-compose -f feagi.yml -f feagi.private.yml up
fi
