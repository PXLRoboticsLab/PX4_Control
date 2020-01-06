# PX4 Control
PX4 Control allows manual offboard control of the PX4 iris drone. This repo also has a [Docker file](#docker) that can be build so that getting up and running goes easier.

## Prerequisite
* Gazebo installed
```
http://gazebosim.org
```
* PX4 Firmware clone and checkout to version v1.8.2
```
cd ~
git clone https://github.com/PX4/Firmware.git
git checkout 34b03d56593815e16a29ba5dcef9b66208285c98
```

## Clone required prackages
```
cd ~/catkin_ws/src
git clone https://bitbucket.org/DataspeedInc/velodyne_simulator.git
cd ~/catkin_ws
catkin build
```

## Installing this package

### Clone this into the src folder of your catkin workspace
```
cd ~/catkin_ws/src
git clone https://github.com/PXLRoboticsLab/PX4_Control.git
```

### Build your catkin workspace
```
cd ~/catkin_ws
catkin build
```

# Single Drone

## Start simulation and link px4

### In the firmware folder of px4
```
cd ~/Firmware
make px4_sitl gazebo
```

### In the your catkin workspace

```
cd ~/catkin_ws
roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557"
```

# Multi drone

## Compile px4 and launch multi uav

### In the firmware folder of px4
```
cd ~/Firmware

git submodule update --init --recursive
DONT_RUN=1 make px4_sitl_default gazebo

source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo
```

### Starting multi uav simulation
```
roslaunch px4 multi_uav_mavros_sitl.launch
```

## Running missions

### If the simulation and link are running open a new terminal and go to your catkin workspace
```
cd ~/catkin_ws
```

### Start the package with
```
rosrun px4_sitl px4_ros.py
```

### Enter a mission file, Example:
```
test.json
multi_test.json
```

## Running manual control
### If the simulation and link are running open a new terminal and go to your catkin workspace
```
cd ~/catkin_ws
```

### Start the manual control
```
rosrun px4_sitl manual_px4.py
```

or
```
rosrun px4_sitl multi_manual_px4.py
```

# Docker
This repo contais a docker file with everything installed that is need to test and use the PX4 control stack. This Docker explanation expects that this git repo get cloned on ~/

## Cloning
```
cd ~
git clone https://github.com/PXLRoboticsLab/PX4_Control.git
```

## Building the docker file
```
cd PX4_Control/docker
./build_image.sh
```

## Running the docker
```
cd PX4_Control/docker
./start_container.sh
```

## Seting up the PX4 Control in the docker
If the Docker is not yet running
```
cd PX4_Control/docker
./start_container.sh
```
Then proceed to the catkin_ws/src and clone the repo.
```
cd Projects/catkin_ws/src
git clone https://github.com/PXLRoboticsLab/PX4_Control.git
cd ..
catkin build
```

## Running the PX4 control stack
Open up 2 terminals.
```
docker exec -it px4_control_docker bash
```

In one terminal start the PX4 stack.
```
roslaunch px4_control multi_uav_mavros_sitl.launch
```

In the other terminal start the PX4 control stack.
```
rosrun px4_control multi_manual_px4_ros.py
```

When asked how many drones the default repo is 1 but its possible to add more drones in the multi_uav_mavros_sitl.launch launch file.
