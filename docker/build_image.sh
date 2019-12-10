#!/bin/bash

(sudo apt install -y mesa-utils)

vendor=`glxinfo | grep vendor | grep OpenGL | awk '{ print $4 }'`

if [ $vendor == "NVIDIA" ]; then
    (docker build -t px4_sitl_docker_base:latest -f ./dockerfile/nvidia/Dockerfile .)
else
    (docker build -t px4_sitl_docker_base:latest -f ./dockerfile/opengl/Dockerfile .)
fi

(docker build -t px4_sitl_docker:latest -f ./dockerfile/px4/Dockerfile .)