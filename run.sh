#!/bin/bash

USERNAME="rosdev"
IMAGE_NAME="ros2_humble_gazebo:dev"
CONTAINER_NAME="ros2_gazebo_dev"
WORKSPACE_DIR="$(pwd)/workspace"
XAUTHORITY="$HOME/.Xauthority"
X11_UNIX="/tmp/.X11-unix"

# Validate paths
[ ! -e "$X11_UNIX" ] && { echo "Error: $X11_UNIX does not exist. Ensure an X server is running."; exit 1; }
[ ! -f "$XAUTHORITY" ] && { echo "Error: $XAUTHORITY does not exist."; exit 1; }
[ ! -d "$WORKSPACE_DIR" ] && { echo "Error: $WORKSPACE_DIR does not exist."; exit 1; }

# Define mounts, environment variables, and runtime flags
MOUNTS=(
    "-v$X11_UNIX:/tmp/.X11-unix:rw"
    "-v$XAUTHORITY:/home/$USERNAME/.Xauthority:rw"
    "-v$WORKSPACE_DIR:/home/$USERNAME/workspace:rw"
    "-v/dev/snd:/dev/snd:rw"
)

ENV_VARS=(
    "-eDISPLAY=$DISPLAY"
    "-eQT_X11_NO_MITSHM=1"
    "-eNVIDIA_DRIVER_CAPABILITIES=all"
)
RUNTIME_FLAGS=(
    "--privileged"
    "--net=host"
    "--ipc=host"
    "--tty"
    "--interactive"
)

# Combine arguments
DOCKER_ARGS=(
    "${RUNTIME_FLAGS[@]}"
    "${ENV_VARS[@]}"
    "${MOUNTS[@]}"
)

# Remove existing container
docker rm -f "$CONTAINER_NAME" 2>/dev/null || true

# Run the container
docker run --name "$CONTAINER_NAME" "${DOCKER_ARGS[@]}" "$IMAGE_NAME" bash 
