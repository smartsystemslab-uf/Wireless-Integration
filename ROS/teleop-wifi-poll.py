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
from geometry_msgs.msg import Twist
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
    serial_reader = serial.Serial('/dev/ttyUL3', 115200, timeout=0.05)
    return serial_reader

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

# main loop
if __name__ == '__main__':
    print('\n***********************************\nThis is the start of the main loop\n***********************************') # TODO for testing purposes
    
    # init ROS node to publish cmd_vel
    ros_ns = rospy.get_namespace()
    rospy.init_node('wifi_poll', anonymous = False)                 # name of the node 
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)     # publishes to the cmd_vel
    print('Node is initialized')

    # TODO insert chip init/restart functions (still needs write functionality)

    # init the serial readers
    serial_reader = initialize_wifi_chip_reader()
    print('Read path to UARTlite2 is initialized')

    # main loop
    try:
        # print start messages
        print(msg)
        #print(vels(speed,turn))
        
        while (1):
            # TODO
            # read UART port periodically
            
            
            app_cmd = serial_reader.read()
            print('Data read from the serial port: ', app_cmd)
            
            # convert command into twist
            twist = convertToTwist(app_cmd)
            print('App command has been converted into Twist object. It looks like:  ', twist)

            # publish twist to cmd_vel topic
            vel_pub.Publish(twist)
            print('Just published the twist object.')

    except:
        # error handling
        print("An error occurred while either while reading the serial port /dev/tty...")


    finally:
        # Stop the motor control just in case 
        app_cmd = 'Stop\n'
        twist = convertToTwist(app_cmd)
        vel_pub.Publish(twist)
