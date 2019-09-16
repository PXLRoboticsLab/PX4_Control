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

state = State()
# mavros /mavros/setpoint_velocity/cmd_vel
# https://github.com/weiweikong/px4_velocity_control_keyboard/blob/master/src/px4_velocity_control_node.cpp
# https://answers.ros.org/question/207097/how-to-send-velocity-to-pixhawk-with-mavros/

def state_callback(data):
    # Make sure that the global object state gets used for to local one.
    global state

    # If there is a change in armed status display it.
    if state.armed != data.armed:
        rospy.loginfo("armed state changed from {0} to {1}"
        .format(state.armed, data.armed))

    # If there is a change in connection status display it.
    if state.connected != data.connected:
        rospy.loginfo("connected changed from {0} to {1}"
        .format(state.connected, data.connected))

    # If there is a change is mode status display it.
    if state.mode != data.mode:
        rospy.loginfo("mode changed from {0} to {1}"
        .format(state.mode, data.mode))

    # If there is a a change in system status display it.
    if state.system_status != data.system_status:
        rospy.loginfo("system_status changed from {0} to {1}"
        .format(mavutil.mavlink.enums['MAV_STATE']
        [state.system_status].name, mavutil.mavlink.enums
        ['MAV_STATE'][data.system_status].name))

    # Set the global status object to the current one we got back from the drone via the topic.
    state = data

def send_heartbeat():
    # Topic to which we have to send the heartbeat to.
    mavlink_pub = rospy.Publisher('mavlink/to', Mavlink, queue_size=1)
    hb_mav_msg = mavutil.mavlink.MAVLink_heartbeat_message(mavutil.mavlink.MAV_TYPE_GCS, 0, 0, 0, 0, 0)
    hb_mav_msg.pack(mavutil.mavlink.MAVLink('', 2, 1))
    hb_ros_msg = mavlink.convert_to_rosmsg(hb_mav_msg)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        mavlink_pub.publish(hb_ros_msg)
        try:  # prevent garbage in console output when thread is killed.
            rate.sleep()
        except rospy.ROSInterruptException as e :
            rospy.logerr( "Heartbeat thread error: {0}".format(e))

if __name__ == '__main__':
    rospy.init_node('test_node', anonymous=True)
    state_sub = rospy.Subscriber('mavros/state', State, state_callback)

    # Start the heartbeat to the drone.
    hb_thread = Thread(target=send_heartbeat, args=())
    hb_thread.daemon = True
    hb_thread.start()
    rospy.Rate(5).sleep()
