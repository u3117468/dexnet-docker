#nvidia-docker create -it --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY --privileged -v /dev/bus/usb:/dev/bus/usb -v /tmp/.X11-unix:/tmp/.X11-unix -v $HOME/.Xauthority:/home/developer/.Xauthority -v /home/declan/Dropbox/RoboticGrasper/test:/apps/test --net=host --pid=host --ipc=host --name dexnetfull-ros-1 dexnet-ros

docker create -it \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -e DISPLAY=unix$DISPLAY \
        --device /dev/dri \
        --privileged \
        -v /dev/bus/usb:/dev/bus/usb \
        -v $HOME/.Xauthority:/home/developer/.Xauthority \
        -v /home/declan/Dropbox/RoboticGrasper/test:/apps/test \
        --net=host --pid=host --ipc=host --name dexnetfull-ros-1 dexnet-ros


docker start dexnetfull-ros-1
docker exec -it dexnetfull-ros-1 bash
