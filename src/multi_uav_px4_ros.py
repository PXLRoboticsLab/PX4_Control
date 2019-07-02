#!/usr/bin/env python
import rospy
import os
import json

from sensor_msgs.msg import Imu
from mavros import mavlink
from mavros_msgs.msg import Mavlink, Waypoint, WaypointReached, State
from mavros_msgs.srv import WaypointPush, SetMode, CommandBool
from pymavlink import mavutil
from threading import Thread
from px4_ros import Px4Ros

def read_file(mission_file_path):
    mission = None
    with open(mission_file_path, 'r') as f:
        mission = json.load(f)

    return mission

def mission_object_constructor(mission_json):
    if 'mission' in mission_json:
        mission_json = mission_json['mission']
    
    if 'items' in mission_json:
        for wp in mission_json['items']:
            yield Waypoint(
                is_current=False,
                frame=int(wp['frame']),
                command=int(wp['command']),
                param1=float('nan'
                             if wp['params'][0] is None else wp['params'][0]),
                param2=float('nan'
                             if wp['params'][1] is None else wp['params'][1]),
                param3=float('nan'
                             if wp['params'][2] is None else wp['params'][2]),
                param4=float('nan'
                             if wp['params'][3] is None else wp['params'][3]),
                x_lat=float(wp['params'][4]),
                y_long=float(wp['params'][5]),
                z_alt=float(wp['params'][6]),
                autocontinue=bool(wp['autoContinue']))
    else:
        raise IOError("no mission items")


if __name__ == '__main__':
    # Ask the user which mission file to read using src/px4_sitl/missions as a base path.
    mission_file = raw_input("Please enter the name of the mission:")
    mission_file_path = os.path.abspath("src/PX4_SITL/missions/" + mission_file)
    rospy.init_node('test', anonymous=True)
    rospy.loginfo(mission_file_path)
    mission = read_file(mission_file_path)
    threads = []
    for key in mission.keys():
        rospy.loginfo(key)
        wps = []
        for waypoint in mission_object_constructor(mission[key]):
            wps.append(waypoint)
            px4ros = Px4Ros(topic_prefix=key, mission=wps)
        threads.append(px4ros)

    for thread in threads:
        thread.start()

    for thread in threads:
        thread.join()