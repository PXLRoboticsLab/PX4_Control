#!/bin/bash

docker stop $(docker ps -aqf "name=px4_sitl_docker")
docker rm $(docker ps -aqf "name=px4_sitl_docker")

xhost +local:docker

# --device=/dev/video0:/dev/video0
# For non root usage:
# RUN sudo usermod -a -G video developer

vendor=`glxinfo | grep vendor | grep OpenGL | awk '{ print $4 }'`

if [ $vendor == "NVIDIA" ]; then
    nvidia-docker run --privileged -it \
        -v `pwd`/src:/home/user/Projects/catkin_ws/src \
        -e XDG_RUNTIME_DIR=/tmp/myxdg \
        -e PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native \
        -e "TERM=xterm-256color" \
        -v ${XDG_RUNTIME_DIR}/pulse/native:${XDG_RUNTIME_DIR}/pulse/native \
                    --group-add $(getent group audio | cut -d: -f3) \
        --device /dev/snd \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        -env="XAUTHORITY=$XAUTH" \
        --volume="$XAUTH:$XAUTH" \
        --runtime=nvidia \
        --cap-add SYS_ADMIN --device /dev/fuse \
        --name px4_sitl_docker \
        px4_sitl_docker:latest \
        bash
else
    docker run --privileged -it \
        -v `pwd`/src:/home/user/Projects/catkin_ws/src \
        --volume=/tmp/.X11-unix:/tmp/.X11-unix \
        --device=/dev/dri:/dev/dri \
        --env="DISPLAY=$DISPLAY" \
        -e "TERM=xterm-256color" \
        --cap-add SYS_ADMIN --device /dev/fuse \
        --name px4_sitl_docker \
        px4_sitl_docker:latest \
        bash
fi
