#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of HolA Bot (KB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			[ 2998 ]
# Author List:		[ Names of team members worked on this file separated by Comma: Mohd Alqama Shaikh, Unmani Shinde, Sameer Gupta, Khushi Balia ]
# Filename:			task_0.py
# Functions:
# 					callback, main
# Nodes:		    turtle_controller


################################# IMPORT MODULES ##################################
import sys
import traceback
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import rospy

############################# Initialized Publisher ##############################

speed_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size = 100)

################# ADD GLOBAL VARIABLES HERE #################

# counter is a flag for execuetion
# counter 2 checks whether we have reached or not to the vertically opposite points
# counter 3 checks whether we have turned by pi/2 after reaching vertically opposite point or not
current_x, current_y, counter, init_x, init_y, counter_2, counter_3 ,theta= 0,0,0,0,0,0,0,0
output = "my TurtleBot is : Moving in circle"
speed = Twist() 
##############################################################


################# ADD UTILITY FUNCTIONS HERE #################
def CalcAng(msg):

    global counter_3
    ##############  Checking whether rotated or not  ##############
    if (-(math.pi/2) - 0.02 <= msg.theta <= -(math.pi/2) + 0.02   ):
        counter_3 += 1

    
def CalcPose(msg):

    global current_x, current_y, counter, init_x, init_y,theta

    ####### Getting the current and initial positions #######
    if (counter == 0):
        init_x = round(msg.x, 2)
        init_y = round(msg.y, 2)
    else:
        current_x = round(msg.x,2)
        current_y = round(msg.y,2)
    counter += 1
    theta = round(msg.theta,2)

##############################################################

def callback(msg):


    global counter_2,output

    CalcPose(msg)

    if (counter != 0):



        if (2 <= current_y-init_y <= 2.03 and 0.021 >= current_x-init_x >= -0.021):
            speed.linear.y = 0
            speed.linear.x = 0
            speed.angular.z = 1
            counter_2 += 1
            output = "my TurtleBot is : Rotating"
            CalcAng(msg)
            if(counter_3 != 0):
                speed.linear.y = 0
                speed.linear.x = 2
                speed.angular.z = 0
                output = "my TurtleBot is : moving Straight"


        elif (counter_2 == 0):
            speed.linear.x = (1)
            speed.angular.z = 1

        elif (counter_2 != 0):
            if (0.02 >= current_y - init_y >= -0.02):
                speed.linear.y = 0
                speed.linear.x = 0
                speed.angular.z = 0

    print(output)
    print( theta) 
    speed_publisher.publish(speed)  
    """
    Purpose: To publish the speed in order to achieve the D in the turtle_sim
    ---
    This function should be used as a callback. Refer Example #1: Pub-Sub with Custom Message in the Learning Resources Section of the Learning Resources.
    You can write your logic here.
    NOTE: Radius value should be 1. Refer expected output in document and make sure that the turtle traces "same" path.

    Input Arguments:
    ---
        `data`  : [Pose] this the information about the current position of the turtle
            data received by the call back function

    Returns: It returns nothing, instead it directly publishes the velocity to the turtle 
    ---
        May vary depending on your logic.

    Example call:
    ---
        Depends on the usage of the function.
    """

def main():

    ###################### Initializing the working node #########################
    rospy.init_node('turtle_controller', anonymous=False)
    ##############################################################################
    
    #################### Subscribing to the relevant topics ######################
    rospy.Subscriber("/turtle1/pose", Pose, callback)
    ##############################################################################

    rospy.spin()
    """
	Purpose: Transferring callback to the publisher
	---
	This function will be called by the default main function given below.
    
    The main functions algorithm is as follows:

    1. It gets all the data of the position of the turtle from the /turtle/pose
    2. Then in the callback function the following sub-algo is applicable:
        1. It recieves msg from the main function from the main function's callback and then calculates simulations current_x and current_y for each iterations and calculates initial x and y for the very first iterations
        2. Now, using this data there are these conditions:
            * For the second iteration (i.e. counter = 1), if we have reached vertically opposite point at the distance of 2 units (Because radius is 1 unit and diameter is 2 units), then we need to stop moving forward and change our orienation by pi/2 (anti-clockwise).
                ** And once we have successfully turned by pi/2 radians, we will then continue to move forward in the turtle's x-axis
            * Else if we have not reached that point yet, then we need to move forward in x by pi (this will give us radius = 1) and rotate by pi radians at the same time.
            * Now, if we have turned by pi/2 anticlockwise and then started moving forward (i.e. counter_2 > 0), then we have to check have we reached the starting point from where we actually started
                ** If yes, then we will stop by giving all the velocities and angular velocities zero values 

	Input Arguments:
	---
        None

	Returns:
	---
        None

	Example call:
	---
        main()
	"""




######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS PART #########
if __name__ == "__main__":
    try:
        print("------------------------------------------")
        print("         Python Script Started!!          ")
        print("------------------------------------------")
        main()

    except:
        print("------------------------------------------")
        traceback.print_exc(file=sys.stdout)
        print("------------------------------------------")
        sys.exit()

    finally:
        print("------------------------------------------")
        print("    Python Script Executed Successfully   ")
        print("------------------------------------------")

