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

class Multi_px4(Thread):
    def __init__(self, topic_prefix=None, actionQueue = queue, *args):
        Thread.__init__(self)
        self.actionQueue = actionQueue
        # Set topics prefix. Will differentiate if it's a multi drone situation.
        # Defaults to single drone.
        if (topic_prefix == "") or (topic_prefix == None):
            self.topic_prefix = ""
        else:
            self.topic_prefix = topic_prefix + '/'



    def run(self):
        self.state = State()
        self.set_arming_srv = rospy.ServiceProxy(self.topic_prefix + 'mavros/cmd/arming', CommandBool)
        self.set_mode_srv = rospy.ServiceProxy(self.topic_prefix + 'mavros/set_mode', SetMode)
        self.mavlink_pub = rospy.Publisher(self.topic_prefix + 'mavlink/to', Mavlink, queue_size=1)
        self.set_raw_local = rospy.Publisher(self.topic_prefix + 'mavros/setpoint_raw/local', PositionTarget, queue_size=100)
        self.state_sub = rospy.Subscriber(self.topic_prefix + 'mavros/state', State, self.state_callback)

        rospy.loginfo("{0}: Starting thread".format(self.topic_prefix))

        self.hb_thread = Thread(target=self.send_heartbeat, args=())
        self.hb_thread.daemon = True
        self.hb_thread.start()

        self.rate = rospy.Rate(20)
        self.att_cmd = PositionTarget()
        self.att_cmd.coordinate_frame = 8
        self.att_cmd.type_mask = 7
        self.att_cmd.velocity = Vector3()

        self.att_cmd.velocity.x = 0
        self.att_cmd.velocity.y = 0
        self.att_cmd.velocity.z = 0.5

        for i in range(0, 100):
            self.set_raw_local.publish(self.att_cmd)
            self.rate.sleep()

        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)

        self.att_cmd.velocity.x = 0
        self.att_cmd.velocity.y = 0
        self.att_cmd.velocity.z = 0.1
        self.att_cmd.yaw = 0

        while not rospy.is_shutdown():
            if not self.actionQueue.empty():
                key = self.actionQueue.get()

                # Left and right.
                if key == "d":
                    self.att_cmd.velocity.x += 0.1
                if key == "q":
                    self.att_cmd.velocity.x -= 0.1

                # Forward and backwards.
                if key == "z":
                    self.att_cmd.velocity.y += 0.1
                if key == "s":
                    self.att_cmd.velocity.y -= 0.1

                # Up and down.
                if key == "a":
                    self.att_cmd.velocity.z += 0.1
                if key == "e":
                    self.att_cmd.velocity.z -= 0.1

                # Yaw left and right.
                if key == "f":
                    self.att_cmd.yaw += 0.1
                if key == "v":
                    self.att_cmd.yaw -= 0.1

                # Stop all movement.
                if key == "r":
                    self.att_cmd.velocity.x = 0
                    self.att_cmd.velocity.y = 0
                    self.att_cmd.velocity.z = 0
                    self.att_cmd.yaw = 0

            self.set_raw_local.publish(self.att_cmd)
            self.rate.sleep()

    def state_callback(self, data):
        # If there is a change in armed status display it.
        if self.state.armed != data.armed:
            rospy.loginfo("armed state changed from {0} to {1}"
            .format(self.state.armed, data.armed))

        # If there is a change in connection status display it.
        if self.state.connected != data.connected:
            rospy.loginfo("connected changed from {0} to {1}"
            .format(self.state.connected, data.connected))

        # If there is a change is mode status display it.
        if self.state.mode != data.mode:
            rospy.loginfo("mode changed from {0} to {1}"
            .format(self.state.mode, data.mode))

        # If there is a a change in system status display it.
        if self.state.system_status != data.system_status:
            rospy.loginfo("system_status changed from {0} to {1}"
            .format(mavutil.mavlink.enums['MAV_STATE']
            [self.state.system_status].name, mavutil.mavlink.enums
            ['MAV_STATE'][data.system_status].name))

        # Set the global status object to the current one we got back from the drone via the topic.
        self.state = data

    def send_heartbeat(self):
        hb_mav_msg = mavutil.mavlink.MAVLink_heartbeat_message(mavutil.mavlink.MAV_TYPE_GCS, 0, 0, 0, 0, 0)
        hb_mav_msg.pack(mavutil.mavlink.MAVLink('', 2, 1))
        hb_ros_msg = mavlink.convert_to_rosmsg(hb_mav_msg)
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.mavlink_pub.publish(hb_ros_msg)
            try:  # prevent garbage in console output when thread is killed.
                rate.sleep()
            except rospy.ROSInterruptException as e :
                rospy.logerr( "Heartbeat thread error: {0}".format(e))

    # Set the drone mode to AUTO.MISSION inorder the execute the mission commands automatically.
    def set_mode(self, mode, timeout):
        # Save the old mode state.
        old_mode = self.state.mode
        loop_freq = 1 # Hz
        rate = rospy.Rate(loop_freq)
        mode_set = False

        # Try setting the mode till the timeout expires or the correct mode is set.
        for i in xrange(timeout * loop_freq):
            if self.state.mode == mode:
                mode_set = True
                rospy.loginfo("set mode success | seconds: {0} of {1}".format(
                i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_mode_srv(0, mode)
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command")
                except rospy.ServiceException as e:
                        print "Service call failed: %s" % e

            try:
                rate.sleep()
            except rospy.ROSException as e:
                print(e)

    def set_arm(self, arm, timeout):
        # Save the old arm state.
        old_arm = self.state.armed
        loop_freq = 1 # Hz
        rate = rospy.Rate(loop_freq)
        arm_set = False
        # Try setting the arm till the timeout expires or the correct arm is set.
        for i in xrange(timeout * loop_freq):
            if self.state.armed == arm:
                arm_set = True
                rospy.loginfo("set arm success | seconds: {0} of {1}".format(
                i / loop_freq, timeout))
                break
            else:
                # Topic to which we have to send the arming command.
                
                try:
                    res = self.set_arming_srv(arm)
                    if not res.success:
                        rospy.logerr("failed to send arm command")
                except rospy.ServiceException as e:
                    print "Service call failed: %s" % e

            try:
                rate.sleep()
            except rospy.ROSException as e:
                print(e)

class keyboardListner(Thread):
    def __init__(self, topic_prefix=None, keyQueue = [], *args):
        Thread.__init__(self)
        self.settings = termios.tcgetattr(sys.stdin)
        self.keyQueue = keyQueue
        self.selectedDrone = 0

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        pressed_key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return pressed_key

    def run(self):
        while not rospy.is_shutdown():
            key = self.get_key()
            # Check if control^c or control^z is pressed.
            if key == '\x03' or key == '\x1A':
                break

            if key == "1":
                self.selectedDrone = 0
            if key == "2":
                self.selectedDrone = 1
            if key == "3":
                self.selectedDrone = 2

            self.keyQueue[self.selectedDrone].put(key)


if __name__ == '__main__':
    print "starting multi manual control"
    rospy.init_node('test_node', anonymous=True)

    droneAmount = int(raw_input("Please enter the amount of drones:"))

    drones = []
    droneMessageQueue = []
    
    for i in range(droneAmount):
        print "Drone %d" % i
        q = queue.Queue(maxsize = 10)
        droneMessageQueue.append(q)
        manualUav = Multi_px4(topic_prefix="uav%d" % i, actionQueue=droneMessageQueue[i])
        drones.append(manualUav)

    print "Starting keyboard thread"

    keyboardThread = keyboardListner(keyQueue = droneMessageQueue)
    keyboardThread.daemon = True
    keyboardThread.start()

    for thread in drones:
        thread.start()