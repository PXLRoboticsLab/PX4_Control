# Specify base image
FROM nvidia/cudagl:9.2-devel-ubuntu18.04

# We love UTF!
ENV LANG C.UTF-8

# Set the environment to noninteractive (no user input)
ENV DEBIAN_FRONTEND noninteractive

# Set NVIDIA variables
ENV NVIDIA_REQUIRE_CUDA "cuda>=8.0"
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,video,display
