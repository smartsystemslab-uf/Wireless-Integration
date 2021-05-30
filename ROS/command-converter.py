#!/usr/bin/env python
# this code is to test a function that takes in commands from the android-app and converts it into Twist commands
# which is what the ROS system uses to drive the motor controller of the bot

from __future__ import print_function
import ropsy
from geometry_msgs.msg import Twist

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
x = 0
y = 0
z = 0
th = 0

# Function declaration
def convertToTwist(cmd):
    if cmd in moveBindings.keys():
        x = moveBindings[cmd][0]
        y = moveBindings[cmd][1]
        z = moveBindings[cmd][2]
        th = moveBindings[cmd][3]
        print('x = ', x, ', ' , 'y = ', y, ', ', 'y = ', z, ', ', 'th = ', th, '\n')    # TODO just for testing purposes will remove
    else:
        print('The word', data, 'does not have an associated moveBinding.')

    #TODO convert x, y, z, th to Twist - need ROS
    # create twist object, assign the correct movement bindings to the twist object, and publish the twist topic
    twist = Twist()
    twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
    return twist
    

# TODO for testing purposes only
convertToTwist("Front\n")
convertToTwist("Left\n")
convertToTwist("Right\n")
convertToTwist("Back\n")
convertToTwist("Stop\n")
convertToTwist("Hi")    # shouldn't print anything
exit()