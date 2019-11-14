# ORB_SLAM2

Not the repo of ORB_SLAM2 itself these are just the launch and config files that we need in order to use it in our project.

## Installing

### Clonning and building
```
cd ~/catkin_ws/src
git clone https://github.com/appliedAI-Initiative/orb_slam_2_ros.git
cd ~/catkin_ws/
catkin build
```

### Running ORB_SLAM2

```
roslaunch px4_sitl orb_slam2_depth.launch
```

## Configuring launch files and config files

### Launch file
In the launch folder of orb_slam2 there is a example file of how a orb_slam2 launch files looks. Inorder to get orb_slam2 working with our depth camera we need to remap the raw image and disparity topics to */camera/rgb/image_raw* and */camera/depth_registered/image_raw* to do so we add.
```
<remap from="/camera/rgb/image_raw" to="your raw image topic" />
<remap from="/camera/depth_registered/image_raw" to="your disparity topic" />

```
point cloud will get published on */orb_slam2_rgbd/map_points*
