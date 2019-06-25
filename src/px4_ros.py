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


class Px4Ros(Thread):
    def __init__(self, topic_prefix=None, mission = [], *args):
        Thread.__init__(self)

        # Set topics prefix. Will differentiate if it's a multi drone situation.
        # Defaults to single drone.
        if (topic_prefix == "") or (topic_prefix == None):
            self.topic_prefix = "mavros"
        else:
            self.topic_prefix = topic_prefix

        rospy.init_node(self.topic_prefix, anonymous=True)

        self.mission_item_reached = -1
        # Mission object
        self.mission = mission

        # Save the state of the drone.
        self.state = State()

        # Prepare all ros topics that will be used.
        self.setUp()

    def setUp(self):
        service_timeout = 30
        rospy.loginfo("Waiting for ROS services.")
        try:
            rospy.wait_for_service(self.topic_prefix + '/set_mode', service_timeout)
            rospy.wait_for_service(self.topic_prefix + '/cmd/arming', service_timeout)
            rospy.wait_for_service(self.topic_prefix + '/mission/push', service_timeout)
            rospy.loginfo("ROS services are up.")
        except rospy.ROSException:
            rospy.loginfo("Failed to connect to service.")

        self.set_mode_srv = rospy.ServiceProxy(self.topic_prefix + '/set_mode', SetMode)
        self.set_arming_srv = rospy.ServiceProxy(self.topic_prefix + '/cmd/arming', CommandBool)
        self.wp_push_srv = rospy.ServiceProxy(self.topic_prefix + '/mission/push', WaypointPush)
        self.state_sub = rospy.Subscriber(self.topic_prefix + '/state', State, self.state_callback)
        self.mavlink_pub = rospy.Publisher(self.topic_prefix + '/to', Mavlink, queue_size=1)

        self.hb_thread = Thread(target=self.heartbeat, args=())
        self.hb_thread.daemon = True
        self.hb_thread.start()

    def run(self):
        self.send_mission()
        self.set_mode("AUTO.MISSION", 5)
        self.set_arm(True, 5)
        while(not self.is_mission_done()):
            self.rospy.Rate(5).sleep()

        self.rospy.spin()

    def state_callback(self, data):
        if self.state.armed != data.armed:
            rospy.loginfo("Armed state change from {0} to {1}"
            .format(self.state.armed, data.armed))

        if self.state.connected != data.connected:
            rospy.loginfo("Connected changed from {0} to {1}").format(self.state.connected, date.connected)

        if self.state.mode != data.mode:
            rospy.loginfo("Mode changed from {0} to {1}").format(self.state.mode, data.mode)

        if self.state.system_status != data.system_status:
            rospy.loginfo("system_status changed from {0} to {1}"
            .format(mavutil.mavlink.enums['MAV_STATE']
            [self.state.system_status].name, mavutil.mavlink.enums
            ['MAV_STATE'][data.system_status].name))

    def heartbeat(self):
        hb_mav_msg = mavutil.mavlink.MAVLink_heartbeat_message(mavutil.mavlink.MAV_TYPE_GCS, 0, 0, 0, 0, 0)
        hb_mav_msg.pack(mavutil.mavlink.MAVLink('', 2, 1))
        hb_ros_msg = mavlink.convert_to_rosmsg(hb_mav_msg)
        rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            self.mavlink_pub.publish(hb_ros_msg)
            try:
                rate.sleep()
            except rospy.ROSInterruptException as e:
                print "Heartbeat thread error: %s" % e

    def send_mission(self):
        try:
            res = self.wp_push_srv(start_index = 0, waypoints = self.mission)
            if res.succes:
                rospy.loginfo("Waypoints successfully transferred")
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def set_arm(self, arm, timeout):
        old_arm = self.state.armed
        loop_freq = 1 #Hz
        rate = rospy.Rate(loop_freq)
        arm_set = False

        for i in xrange(timeout * loop_freq):
            if self.state.armed == arm:
                arm_set = True 
                rospy.loginfo("Set arm sccuess | seconds: {0} of {1}"
                .format(i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_arming_srv(arm)
                    if not res.success:
                        rospy.logerr("Failed to send arm command")
                except rospy.ServiceException as e:
                    print "Service call failed: %s" % e

            try:
                rate.sleep()
            except rospy.ROSException as e:
                print e

    def set_mode(self, mode, timeout):
        old_mode = self.state.mode
        loop_freq = 1 #Hz
        rospy.Rate(loop_freq)
        mode_set = False

        for i in xrange(timeout * loop_freq):
            if self.state.mode == mode:
                mode_set = True
                rospy.loginfo("Set mode succes | seconds: {0} to {1}"
                .format(i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_mode_srv(0, mode)
                    if not res.mode_send:
                        rospy.logerr("Failed to send mode command")
                except rospy.ServiceException as e:
                    print "Service call failed: %s" % e

            try:
                rate.sleep()
            except rospy.ServiceException as e:
                print e

    def is_mission_done(self):
        if self.mission_item_reached == len(self.mission):
            return True
        else:
            return False

    def mission_item_reached_callback(self, data):
        if self.mission_item_reached != data.wp_seq:
            rospy.loginfo("mission item reached: {0}".format(data.wp_seq))
            self.mission_item_reached = data.wp_seq