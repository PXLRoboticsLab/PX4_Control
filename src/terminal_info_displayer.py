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

ter_height = 32
ter_width = 100
def infoPrinter():
    sys.stdout.write("\x1b[8;{rows};{cols}t".format(rows=ter_height, cols=ter_width))
    sys.stdout.flush()
    os.system('clear')
    os.system('setterm -cursor off')
    sys.stdout.flush()
    speed = 0
    alltiude = 1
    while True:

        '''
        sys.stdout.write(u"\r\u250c\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2510\n")
        sys.stdout.write(u"\u2502 Doing thing %i \u2502\n" % i)
        sys.stdout.write(u"\r\u251c\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2524\n")
        sys.stdout.write(u"\r\u2502 Doing thing %i \u2502 \n" % i)
        sys.stdout.write(u"\r\u251c\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2524\n")
        sys.stdout.write(u"\r\u2502 Doing thing %i \u2502 \n" % i)
        sys.stdout.write(u"\r\u251c\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2524\n")
        sys.stdout.write(u"\r\u2502 Doing thing %i \u2502 \n" % i)
        sys.stdout.write(u"\r\u251c\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2524\n")
        sys.stdout.write(u"\r\u2502 Doing thing %i \u2502 \n" % i)
        sys.stdout.write(u"\r\u251c\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2524\n")
        sys.stdout.write(u"\r\u2502 Doing thing %i \u2502 \n" % i)
        sys.stdout.write(u"\r\u2514\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2518\n")
        '''

        sys.stdout.write(u"\u250f")
        for i in range(ter_width-2):
            sys.stdout.write(u"\u2501")
        sys.stdout.write(u"\u2513\n")

        for i in range(ter_height-2):
            sys.stdout.write(u"\u2503")
            for i in range(ter_width-2):
                sys.stdout.write(u" ")
            sys.stdout.write(u"\u2503\n")

        sys.stdout.write(u"\u2517")
        for i in range(ter_width-2):
            sys.stdout.write(u"\u2501")
        sys.stdout.write(u"\u251b")

        sys.stdout.write("\033[%d;%dH" % (2, 2))
        sys.stdout.write(u"Drone speed %s" % speed)
        sys.stdout.write("\033[%d;%dH" % (3, 2))
        sys.stdout.write(u"altitude %s" % alltiude)
        sys.stdout.write("\033[%d;%dH" % (0, 0))
        sys.stdout.flush()
        speed += 0.1
        alltiude += 0.1
        time.sleep(0.5)
    os.system('clear')
    os.system('clear')


if __name__ == '__main__':
    info_thread = Thread(target=infoPrinter, args=())
    info_thread.start()
    info_thread.join()