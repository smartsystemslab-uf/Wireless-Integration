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

################################################## Dependencies ####################################################

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
#from commandConverter import convertToTwist        # see command-converter.py

msg = """
Starting WIFI Node...

Reading the WIFI Chip UART

Press CTRL+C to quit.
"""

################################################## Function Declarations ###########################################

# Associates the commands from the mobile to movement bindings using the format: 'CommandFromApp\n' : (x, y, z, th), 
moveBindings = {
    'Stop\n'        : (0,0,0,0),
    'Front\n'       : (1,0,0,0),        # TODO change to "Forward"
    'UpRight\n'     : (1,0,0,-1),
    'Left\n'        : (0,0,0,1),
    'Right\n'       : (0,0,0,-1),
    'UpLeft\n'      : (1,0,0,1),
    'Back\n'        : (-1,0,0,0),       # TODO change to "Backward"
    'DownRight\n'   : (-1,0,0,1),
    'DownLeft\n'    : (-1,0,0,-1),
}

# Associates the commands from the mobile app to speed bindings
speedBindings={
    'q':(1.1,1.1),          # TODO change these keys to 'SpeedInc', 'SpeedDec', etc
    'z':(.9,.9),
    'w':(1.1,1),
    'x':(.9,1),
    'e':(1,1.1),
    'c':(1,.9),
}

# Initiliaze the variables
# TODO figure out what get_param does 
speed = rospy.get_param("~speed", 0.5)
turn = rospy.get_param("~turn", 1.0)

# Function Declarations 
def convertToTwist(cmd):
    x = 0
    y = 0
    z = 0
    th = 0

    if cmd in moveBindings.keys():
        x = moveBindings[cmd][0]
        y = moveBindings[cmd][1]
        z = moveBindings[cmd][2]
        th = moveBindings[cmd][3]
        #print('x = ', x, ', ' , 'y = ', y, ', ', 'y = ', z, ', ', 'th = ', th, '\n')    # TODO just for testing purposes will remove
    elif not cmd:
        print('The received command is null')
    else:
        print('The word', cmd, 'does not have an associated moveBinding.')

    #TODO convert x, y, z, th to Twist - need ROS
    # create twist object, assign the correct movement bindings to the twist object, and publish the twist topic
    twist = Twist()
    twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
    #print(twist)
    return twist

def initialize_wifi_chip_reader():
    # TODO figure the correct UART path
    serial_reader = serial.Serial('/dev/ttyUL3', 115200, timeout=0.05)
    return serial_reader

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

####################################################################################################################



# main loop
if __name__ == '__main__':
    
    # init ROS node to publish cmd_vel
    print('\n***********************************\nThis is the start of the main loop\n***********************************\n') # TODO for testing purposes
    ros_ns = rospy.get_namespace()
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)     # publishes to the cmd_vel
    rospy.init_node('wifi_poll', anonymous = False)                 # name of the node 
    print('Node is initialized')

    # TODO insert chip init/restart functions (still needs write functionality)

    # init the serial readers
    serial_reader = initialize_wifi_chip_reader()
    print('Read path to UARTlite2 is initialized')

    # main loop
    try:
        # print start messages
        print(msg)
        print(vels(speed,turn))
        
        while (1):
            # TODO
            # read UART port periodically
            # trying dif values
            app_cmd = serial_reader.read(100)
            print('Data read from the serial port: ', app_cmd)

            # convert command into twist
            twist = convertToTwist(app_cmd)
            print('App command has been converted into Twist object. It looks like:  ', twist)

            # publish twist to cmd_vel topic
            vel_pub.publish(twist)
            print('Just published the twist object.')

    except Exception as e:
        # error handling
        print(e)
        print("An error occurred while either while reading the serial port /dev/tty...")


    finally:
        # just in case do it again
        twist = Twist()
        twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
        vel_pub.publish(twist)
