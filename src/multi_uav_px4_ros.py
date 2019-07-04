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

def heartbeat(topic_prefix):
    mavlink_pub = rospy.Publisher(topic_prefix + '/mavlink/to', Mavlink, queue_size=1)
    hb_mav_msg = mavutil.mavlink.MAVLink_heartbeat_message(mavutil.mavlink.MAV_TYPE_GCS, 0, 0, 0, 0, 0)
    hb_mav_msg.pack(mavutil.mavlink.MAVLink('', 2, 1))
    hb_ros_msg = mavlink.convert_to_rosmsg(hb_mav_msg)
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        mavlink_pub.publish(hb_ros_msg)
        print "Tick tock heartbeat uav: %s " % topic_prefix
        try:
            rate.sleep()
        except rospy.ROSInterruptException as e:
            print "Heartbeat thread error: %s" % e


if __name__ == '__main__':
    # Ask the user which mission file to read using src/px4_sitl/missions as a base path.
    mission_file = raw_input("Please enter the name of the mission:")
    mission_file_path = os.path.abspath("src/PX4_SITL/missions/" + mission_file)
    rospy.init_node('test', anonymous=True)
    rospy.loginfo(mission_file_path)
    mission = read_file(mission_file_path)
    threads = []
    heartbeat = []

    for key in mission.keys():
        rospy.loginfo(key)
        wps = []
        for waypoint in mission_object_constructor(mission[key]):
            wps.append(waypoint)
            px4ros = Px4Ros(topic_prefix=key, mission=wps)
        threads.append(px4ros)

        heart_beat_thread = Thread(target=heartbeat, args=(key,))
        heart_beat_thread.daemon = False
        heart_beat_thread.start()

    rospy.Rate(5).sleep()

    for thread in threads:
        thread.start()

    for thread in threads:
        thread.join()

    rospy.spin()