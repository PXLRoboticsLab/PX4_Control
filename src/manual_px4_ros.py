#!/usr/bin/env python
import rospy
import os
import json
import time
import select
import sys
import termios
import tty
import queue

from sensor_msgs.msg import Imu
from mavros import mavlink
from mavros_msgs.msg import Mavlink, Waypoint, WaypointReached, State, AttitudeTarget, PositionTarget
from mavros_msgs.srv import WaypointPush, SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, Vector3, Pose
from std_msgs.msg import Float32, Float64

from pymavlink import mavutil
from threading import Thread

state = State()
q = queue.Queue(maxsize = 10)
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
    mavlink_pub = rospy.Publisher('uav1/mavlink/to', Mavlink, queue_size=1)
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
            set_mode_srv = rospy.ServiceProxy('uav1/mavros/set_mode', SetMode)
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
            set_arming_srv = rospy.ServiceProxy('uav1/mavros/cmd/arming', CommandBool)
            try:
                res = set_arming_srv(arm)
                if not res.success:
                    rospy.logerr("failed to send arm command")
            except rospy.ServiceException as e:
                print "Service call failed: %s" % e

        try:
            rate.sleep()
        except rospy.ROSException as e:
            print(e)

def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    pressed_key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return pressed_key

def keyboardListener():
    while not rospy.is_shutdown():
        key = get_key()
        # Check if control^c or control^z is pressed.
        if key == '\x03' or key == '\x1A':
            break

        q.put(key)


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('test_node', anonymous=True)
    state_sub = rospy.Subscriber('uav1/mavros/state', State, state_callback)

    # Start the heartbeat to the drone.
    hb_thread = Thread(target=send_heartbeat, args=())
    hb_thread.daemon = True
    hb_thread.start()
    rospy.Rate(5).sleep()

    kb_thread = Thread(target=keyboardListener, args=())
    kb_thread.daemon = True
    kb_thread.start()

    set_raw_local = rospy.Publisher('uav1/mavros/setpoint_raw/local', PositionTarget, queue_size=100)

    '''
    set_att_pub = rospy.Publisher('mavros/setpoint_attiude/attitude', PoseStamped, queue_size=100)
    set_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    set_raw_att = rospy.Publisher('mavros/setpoint_raw/atittude', AttitudeTarget, queue_size=100)
    '''

    rate = rospy.Rate(20)

    att_cmd = PositionTarget()
    att_cmd.coordinate_frame = 8
    att_cmd.type_mask = 7
    att_cmd.velocity = Vector3()
    att_cmd.velocity.x = 0
    att_cmd.velocity.y = 0
    att_cmd.velocity.z = 0.5

    for i in range(0, 100):
        set_raw_local.publish(att_cmd)
        rate.sleep()

    set_mode("OFFBOARD", 5)
    set_arm(True, 5)

    att_cmd.velocity.x = 0
    att_cmd.velocity.y = 0
    att_cmd.velocity.z = 0.1
    att_cmd.yaw = 0

    while not rospy.is_shutdown():
        if not q.empty():
            key = q.get()

            # Left and right.
            if key == "d":
                att_cmd.velocity.x += 0.1
            if key == "q":
                att_cmd.velocity.x -= 0.1

            # Forward and backwards.
            if key == "z":
                att_cmd.velocity.y += 0.1
            if key == "s":
                att_cmd.velocity.y -= 0.1

            # Up and down.
            if key == "a":
                att_cmd.velocity.z += 0.1
            if key == "e":
                att_cmd.velocity.z -= 0.1

            # Yaw left and right.
            if key == "f":
                att_cmd.yaw += 0.1
            if key == "v":
                att_cmd.yaw -= 0.1

            # Stop all movement.
            if key == "r":
                att_cmd.velocity.x = 0
                att_cmd.velocity.y = 0
                att_cmd.velocity.z = 0
                att_cmd.yaw = 0

        set_raw_local.publish(att_cmd)
        rate.sleep()

    '''
    att_cmd = PoseStamped()
    att_cmd.pose = Pose()
    att_cmd.pose.position.x = 0
    att_cmd.pose.position.y = 0
    att_cmd.pose.position.z = 2
    for i in range(0,100):
        set_pos_pub.publish(att_cmd)
        rate.sleep()

    set_mode("OFFBOARD", 5)
    set_arm(True, 5)

    att_cmd.pose.position.y = 1
    while not rospy.is_shutdown():
        att_cmd.pose.position.y += 0.1
        set_pos_pub.publish(att_cmd)
        rate.sleep()
    '''

    '''
    att_cmd = PoseStamped()
    att_cmd.pose = Pose()
    att_cmd.pose.position.x = 0
    att_cmd.pose.position.y = 0
    att_cmd.pose.position.z = 2

    for i in range(0,100):
        set_att_pub.publish(att_cmd)
        set_thr_pub.publish(Float32(0.5))
        rate.sleep()

    set_mode("OFFBOARD", 5)
    set_arm(True, 5)

    while not rospy.is_shutdown():

        print 'Sending att and thr commands'
        set_att_pub.publish(att_cmd)
        set_thr_pub.publish(Float32(0.5))

        rate.sleep()
    '''