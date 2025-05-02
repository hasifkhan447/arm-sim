#!/bin/bash

docker build -t ros2_humble_gazebo:dev --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) -f docker/Dockerfile .

