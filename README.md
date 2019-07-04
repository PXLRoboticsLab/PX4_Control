# SITL gazebo

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

## Start simulation and link px4

### In the firmware folder of px4
```
cd ~/Firmware
make posix_sitl_default gazebo
```

### In the your catkin workspace

```
cd ~/catkin_ws
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

## Running this package

### If the simulation and link are running open a new terminal and go to your catkin workspace
```
cd ~/catkin_ws
```

### Start the package with
````
rosrun px4_sitl px4_ros.py
````

### Enter a mission file, there is a default mission file called test.json
