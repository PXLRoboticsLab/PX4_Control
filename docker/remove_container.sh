#!/bin/bash

docker stop $(docker ps -aqf "name=px4_sitl_docker")
docker rm $(docker ps -aqf "name=px4_sitl_docker")
