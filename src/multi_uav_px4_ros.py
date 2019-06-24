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




