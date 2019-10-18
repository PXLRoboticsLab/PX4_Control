# PX4 SITL gazebo

## Prerequisite
* Gazebo installed
```
http://gazebosim.org
```
* PX4 Firmware clone and checkout to version v1.8.2
```
cd ~
git clone https://github.com/PX4/Firmware.git
git checkout tags/v1.8.2
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
git clone https://github.com/PXLRoboticsLab/PX4_SITL.git
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