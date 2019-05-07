# SITL gazebo

## Installing this package

### Clone this into the src folder of your catkin workspace
### Build your catkin workspace

## Start simulation and link px4

### In the firmware folder of px4
```
make px4_sitl_default gazebo
```

### In the your catkin workspace

```
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

## Running this package

## If the simulation and link are running open a new terminal and go to your catkin workspace
```
cd ~/catkin_ws
```

## Start the package with
````
rosrun px4_sitl px4_ros.py
````

## Enter a mission file, there is a default mission file called test.json
