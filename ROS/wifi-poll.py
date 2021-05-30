#!/usr/bin/env python
# 
# File:         wifi-chip-interface.py
# Author:       Edward Devlin
# Date:         May 24, 2021
# Description:  Python (2.7) script that interface the ports of the 
#               Zybo Z7 dev board for serial communication. This 
#               script is intended to interface the wifi chip module
#               ESP8266 to the Zybo to allow wifi communications. This
#               script reads packets of wifi from the chip and filters
#               for the cmd_vel to publish to that rostopic
#
# Topics:       Publishes cmd_vel topic
# 
# Notes:
#       look for constraint file something.xdc
#       JE
#       use UL3 (uartlite 2) V13 U17 from .xdc
#       May need to split into two nodes - 1 wifi poller, 1 packet filterer

# Dependencies copied from teleop_twist_keyboard.py
from __future__ import print_function
import rospy

# ROS Dependencies copied from motor-controller-interface.py
import math
from math import sin, cos, pi                                           # 
import re 
import tf

# Core Dependencies
import serial
import time
import mmap

msg = """
Starting WIFI Node...

Reading the WIFI Chip UART

Press CTRL+C to quit.
"""

# Function Declarations 
def initialize_wifi_chip_reader():
    # TODO figure the correct UART path
    serial_reader = serial.Serial('/dev/ttyUL3', 115200, timout=0.05)
    return serial_reader

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

# main loop
if __name__ == '__main__':
    
    # init ROS node to publish cmd_vel
    ros_ns = rospy.get_namespace()
    rospy.init_node('wifi_poll', anonymous = False)                 # name of the node 
    vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 1)     # publishes to the cmd_vel

    # TODO understand how get_param works and what it does
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1)
    
    # TODO insert chip init/restart functions

    # init the serial readers and writers
    serial_reader = initialize_wifi_chip_reader()
    serial_writer = initialize_wifi_chip_writer()

    # create variables to store movement data
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    # main loop
    try:
        # print start messages
        print(msg)
        print(vels(speed,turn))
        
        while (1):
            # TODO
            # read UART port periodically
                # Bobda say use polling or check an interrupt flag, or use daemon look it up

            # filter through packet for the velocity command

            # convert velocity command into a movement binding

            # create twist object, assign the correct movement bindings to the twist object, and publish the twist topic
            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
            vel_pub.Publish(twist)

    except:
        # error handling
        print("An error occurred while either while trying to interperet the wifi packets...")

    finally:
        # just in case do it again
        twist = Twist()
        twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
        vel_pub.Publish(twist)
