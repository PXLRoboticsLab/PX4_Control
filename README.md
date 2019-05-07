# SITL gazebo

## Start simulation and link px4

### In the firmware folder of px4
'''
make px4_sitl_default gazebo
'''

### In the your catkin workspace

'''
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
'''