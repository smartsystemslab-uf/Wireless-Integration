#!/usr/bin/env python
# 
# File:         wifi-chip-interface.py
# Author:       Edward Devlin
# Date:         May 24, 2021
# Description:  Python (2.7) script that interface the ports of the 
#               Zybo Z7 dev board for serial communication. This 
#               script is intended to interface the wifi chip module
#               ESP8266 to the Zybo to allow wifi communications. This
#               script polls a UART on the Zybo attached to the ESP8266, 
#               converts the command data into a ROS datatype, and publishes
#               to the cmd_vel topic
#
# Notes:        lLook for constraint file something.xdc to find and available serial line
#               The one this uses PMOD JE using UL3 (uartlite 2) pins V13 U17 (see .xdc file)

# Dependencies copied from teleop_twist_keyboard.py
from __future__ import print_function
import rospy
from geometry_msgs.msgs import Twist
from std_msgs.msg import String 
import sys, select, termios, tty

# ROS Dependencies copied from motor-controller-interface.py
import math
from math import sin, cos, pi                                           # 
import re 
import tf

# Core Dependencies
import serial
import time
import mmap
from commandConverter import convertToTwist        # see command-converter.py

msg = """
Starting WIFI Node...

Reading the WIFI Chip UART

Press CTRL+C to quit.
"""

# Function Declarations 
def initialize_wifi_chip_reader():
    # TODO figure the correct UART path
    serial_reader = serial.Serial(read_path, 115200, timout=0.05)
    return serial_reader

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

# main loop
if __name__ == '__main__':
    
    # init ROS node to publish cmd_vel
    ros_ns = rospy.get_namespace()
    rospy.init_node('wifi_poll', anonymous = False)                 # name of the node 
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)     # publishes to the cmd_vel
    
    # TODO insert chip init/restart functions (still needs write functionality)

    # init the serial readers
    read_path = '/dev/ttyUL3'
    serial_reader = initialize_wifi_chip_reader()

    # main loop
    try:
        # print start messages
        print(msg)
        print(vels(speed,turn))
        
        while (1):
            # TODO
            # read UART port periodically
            app_cmd = serial_reader.read()
            print("The command read is: ", app_cmd)

            # convert command into twist
            twist = convertToTwist(app_cmd)

            # publish twist to cmd_vel topic
            vel_pub.Publish(twist)

    except:
        # error handling
        print("An error occurred while either while reading the serial port /dev/tty...")


    finally:
        # just in case do it again
        twist = Twist()
        twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
        vel_pub.Publish(twist)
