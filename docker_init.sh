#!/bin/bash

export DISPLAY=:1
# export XDG_RUNTIME_DIR=/tmp/runtime-root
xhost +local:docker
docker run -it --rm --name lidar_filtering --net=host -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix -v ${PWD}:/workspace lidar_filtering bash -c "roscore & > /dev/null & sleep 2; bash"

