# this code is to test a function that takes in commands from the android-app and converts it into Twist commands
# which is what the ROS system uses to drive the motor controller of the bot

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
x = 0
y = 0
z = 0
th = 0

# Function declaration
def convertToTwist(data):
    if data in moveBindings.keys():
        x = moveBindings[data][0]
        y = moveBindings[data][1]
        z = moveBindings[data][2]
        th = moveBindings[data][3]
        print('x = ', x, ', ' , 'y = ', y, ', ', 'y = ', z, ', ', 'th = ', th, '\n')    # TODO just for testing purposes will remove
    else:
        print('The word', data, 'does not have an associated moveBinding.')

    #TODO convert x, y, z, th to Twist - need ROS

    
convertToTwist("Front\n")
convertToTwist("Left\n")
convertToTwist("Right\n")
convertToTwist("Back\n")
convertToTwist("Stop\n")
convertToTwist("Hi")    # shouldn't print anything
exit()