#!/bin/bash

#add the location of the dexnet docker github repo
repoDirectory=''


docker create -it \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -e DISPLAY=unix$DISPLAY \
        --device /dev/dri \
        --privileged \
        -v /dev/bus/usb:/dev/bus/usb \
        -v $HOME/.Xauthority:/home/developer/.Xauthority \
        -v $repoDirectory:/apps/test \
        --net=host --pid=host --ipc=host --name dexnet-ros-cont dexnet-ros


docker start dexnet-ros-cont
docker exec -it dexnet-ros-cont bash
