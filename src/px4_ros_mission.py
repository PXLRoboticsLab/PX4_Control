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
# Execute the mission that has been specified.
def mission(mission_file_path):
    try:
        mission = read_mission(mission_file_path)
    except IOError as e:
        print e
    try:
        # Topic to which we have to push the waypoint array to.
        wp_push_srv = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
        res = wp_push_srv(start_index = 0, waypoints = mission)
        if res.success:
            rospy.loginfo("waypoints successfully transferred")
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

# Make sure the drone still thinks it has a connection to the base station.
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
            print "Heartbeat thread error: %s" % e

# Read in the mission file.
def read_mission(mission_file_path):
    wps = []
    with open(mission_file_path, 'r') as f:
        for waypoint in read_plan_file(f):
            wps.append(waypoint)
            rospy.logdebug(waypoint)

    # set first item to current.
    if wps:
        wps[0].is_current = True

    return wps

# Read in the waypoints and send them back in a Waypoints object.
def read_plan_file(f):
    d = json.load(f)
    if 'mission' in d:
        d = d['mission']
    # For each item in the item array return a new Waypoints object.
    # See documentation for what command 16 (MAV_CMD_NAV_WAYPOINT) does.
    # https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT

    # Command 22 is takeoff.    MAV_CMD_NAV_TAKEOFF
    # Command 21 is land.       MAV_CMD_NAV_LAND
    # Command 16 is waypoint.   MAV_CMD_NAV_WAYPOINT
    if 'items' in d:
        for wp in d['items']:
            # Mission item.
            # Documentation: https://mavlink.io/en/messages/common.html#MISSION_ITEM_INT
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
                param4=float('nan control'
                             if wp['params'][3] is None else wp['params'][3]),
                x_lat=float(wp['params'][4]),
                y_long=float(wp['params'][5]),
                z_alt=float(wp['params'][6]),
                autocontinue=bool(wp['autoContinue']))
    else:
        raise IOError("no mission items")

# Set the drone mode to AUTO.MISSION inorder the execute the mission commands automatically.
def set_mode(mode, timeout):
    # Make sure that the global object state gets used for to local one.
    global state

    # Save the old mode state.
    old_mode = state.mode
    loop_freq = 1 # Hz
    rate = rospy.Rate(loop_freq)
    mode_set = False

    # Try setting the mode till the timeout expires or the correct mode is set.
    for i in xrange(timeout * loop_freq):
        if state.mode == mode:
            mode_set = True
            rospy.loginfo("set mode success | seconds: {0} of {1}".format(
                i / loop_freq, timeout))
            break
        else:  
            # Topic to which we have to send the mode command.
            set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
            try:
                res = set_mode_srv(0, mode)
                if not res.mode_sent:
                    rospy.logerr("failed to send mode command")
            except rospy.ServiceException as e:
                    print "Service call failed: %s" % e

        try:
            rate.sleep()
        except rospy.ROSException as e:
            print(e)

# Arm the drone and making it ready for takeoff.
def set_arm(arm, timeout):
    # Make sure that the global object state gets used for to local one.
    global state

    # Save the old arm state.
    old_arm = state.armed
    loop_freq = 1 # Hz
    rate = rospy.Rate(loop_freq)
    arm_set = False
    # Try setting the arm till the timeout expires or the correct arm is set.
    for i in xrange(timeout * loop_freq):
        if state.armed == arm:
            arm_set = True
            rospy.loginfo("set arm success | seconds: {0} of {1}".format(
            i / loop_freq, timeout))
            break
        else:
            # Topic to which we have to send the arming command.
            set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            try:
                res = set_arming_srv(arm)
                if not res.success:
                    rospy.logerr("failed to send arm command")
            except rospy.ServiceException as e:
                print "Service call failed: %s" % e

        try:
            rate.sleep()
        except rospy.ROSException as e:
            print e

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

if __name__ == '__main__':
    rospy.init_node('test_node', anonymous=True)

    # Ask the user which mission file to read using src/px4_sitl/missions as a base path.
    mission_file = raw_input("Please enter the name of the mission:")
    mission_file_path = os.path.abspath("src/PX4_SITL/missions/" + mission_file)
    print mission_file_path

    # Start listening to the sate of the drone.
    state_sub = rospy.Subscriber('mavros/state', State, state_callback)

    # Start the heartbeat to the drone.
    hb_thread = Thread(target=send_heartbeat, args=())
    hb_thread.daemon = True
    hb_thread.start()
    rospy.Rate(5).sleep()

    # Send the mission.
    mission(mission_file_path)

    # Set the mission type.
    set_mode("AUTO.MISSION", 5)

    # Arm the drone.
    set_arm(True, 5)

    rospy.spin()