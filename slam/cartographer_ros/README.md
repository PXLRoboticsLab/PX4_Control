# Cartographer_ros

Not the repo of Cartographer itself these are just the launch and config files that we need in order to use it in our project.

## Installing
For the complete tutorial https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html

```
cd ~/catkin_ws/

sudo apt-get update
sudo apt-get install -y python-wstool python-rosdep ninja-build
```
Create a new cartographer_ros workspace in ‘catkin_ws’.
```
mkdir catkin_ws
cd catkin_ws
wstool init src
wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src
```
Install cartographer_ros’ dependencies (proto3 and deb packages). The command ‘sudo rosdep init’ will print an error if you have already executed it since installing ROS. This error can be ignored.
```
src/cartographer/scripts/install_proto3.sh
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
```
Build and install.
```
catkin_make_isolated --install --use-ninja
```

If you get errors after using catkin_make_isolated run in the catkin_ws folder
```
catkin build
```

## Running Cartographer

```
roslaunch px4_sitl cartographer_vlp-16.launch
```

