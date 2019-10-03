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

class multi_px4(Thread):
    def __init__(self, topic_prefix=None, actionQueue = queue, *args):
        Thread.__init__(self)

        # Set topics prefix. Will differentiate if it's a multi drone situation.
        # Defaults to single drone.
        if (topic_prefix == "") or (topic_prefix == None):
            self.topic_prefix = ""
        else:
            self.topic_prefix = topic_prefix + '/'

        self.state = State()

if __name__ == '__main__':
    print "starting multi manual control"