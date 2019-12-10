# Specify base image
FROM ubuntu:18.04

# We love UTF!
ENV LANG C.UTF-8

# Set the environment to noninteractive (no user input)
ENV DEBIAN_FRONTEND noninteractive

# Install OpenGL packages
RUN set -x \
        && apt-get update \
        && apt-get install -y xserver-xorg-video-all libgl1-mesa-glx libgl1-mesa-dri \
        && rm -rf /var/lib/apt/lists/*
