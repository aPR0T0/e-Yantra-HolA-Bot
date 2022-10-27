#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of HolA Bot (HB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:		[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		feedback.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node


################### IMPORT MODULES #######################

from glob import glob
from turtle import clear
import rospy
import signal		# To handle Signals by OS/user
import sys		# To handle Signals by OS/user

from geometry_msgs.msg import Wrench		# Message type used for publishing force vectors
from geometry_msgs.msg import PoseArray	# Message type used for receiving goals
from geometry_msgs.msg import Pose2D		# Message type used for receiving feedback

import numpy as np
import time
import math		# If you find it useful

from tf.transformations import euler_from_quaternion	# Convert angles

################## GLOBAL VARIABLES ######################

PI = 3.14

x_goals 		  =  	[]
y_goals 		  =  	[]
theta_goals		  =	 	[]
prev_x_goals	  = [0,0,0,0,0]
prev_y_goals      = [0,0,0,0,0]
prev_theta_goals  = [0,0,0,0,0]

right_wheel_pub = None
left_wheel_pub = None
front_wheel_pub = None

# Initializing the variables
hola_x = 0
hola_y = 0
hola_theta = 0
vel_x = 0
vel_y = 0
vel_z = 0
Helper_time = 0
index = 0
flag = 0

# Initialise variables that may be needed for the control loop
# For ex: x_d, y_d, theta_d (in **meters** and **radians**) for defining desired goal-pose.
des_x = 0
des_y = 0
des_theta = 0

# and also Kp values for the P Controller
kp_x = 2
kp_y = 2
kp_theta = 5 #initializing kp

errors = np.array([[0],[0],[0]])

##################### FUNCTION DEFINITIONS #######################

# NOTE :  You may define multiple helper functions here and use in your code

def signal_handler(sig, frame):
	
	# NOTE: This function is called when a program is terminated by "Ctr+C" i.e. SIGINT signal 	
	print('Clean-up !')
	cleanup()
	sys.exit(0)

def cleanup(err_x, err_y, err_z):
	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Not mandatory - but it is recommended to do some cleanup over here,
	#	   to make sure that your logic and the robot model behaves predictably in the next run.

	############################################
	global rotation_matrix, errors
	rotation_matrix = np.array([[math.cos(0)*math.cos(hola_theta),math.sin(hola_theta)*math.cos(0),-math.sin(0)],
								[math.sin(0)*math.sin(0)*math.cos(hola_theta)-math.cos(0)*math.sin(hola_theta),
								math.sin(0)*math.sin(0)*math.sin(hola_theta)+math.cos(0)*math.cos(hola_theta),
								math.sin(0)*math.cos(0)],
								[math.cos(0)*math.sin(0)*math.cos(hola_theta)+math.sin(0)*math.sin(hola_theta),
								math.cos(0)*math.sin(0)*math.sin(hola_theta)-math.sin(0)*math.cos(hola_theta),
								math.cos(0)*math.cos(0)]]) # rotation matrix from inertial to body
	# (Calculate error in body frame)
	# But for Controller outputs robot velocity in robot_body frame, 
	# i.e. velocity are define is in x, y of the robot frame, 
	# Notice: the direction of z axis says the same in global and body frame
	# therefore the errors will have have to be calculated in body frame.
	errors = np.array([[err_x],[err_y],[err_z]]) # errors in body frame
	errors = np.matmul(rotation_matrix,errors) # errors in inertial frame

def task2_goals_Cb(msg):
	global x_goals, y_goals, theta_goals, prev_x_goals, prev_y_goals, prev_theta_goals
    
	x_goals.clear()
	y_goals.clear()
	theta_goals.clear()

	for waypoint_pose in msg.poses:
		x_goals.append(waypoint_pose.position.x)
		y_goals.append(waypoint_pose.position.y)

		orientation_q = waypoint_pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		theta_goal = euler_from_quaternion (orientation_list)[2]
		theta_goals.append(theta_goal)

	if(len(x_goals) == 0 or len(y_goals) == 0 or len(theta_goals) == 0):
		rospy.Subscriber('task1_goals', PoseArray, task2_goals_Cb)	
	else:
		prev_x_goals, prev_y_goals, prev_theta_goals = x_goals, y_goals, theta_goals


def aruco_feedback_Cb(msg):
	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Receive & store the feedback / coordinates found by aruco detection logic.
	#	-> This feedback plays the same role as the 'Odometry' did in the previous task.

	############################################
	clear()# comment or remove this

def inverse_kinematics():
	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Use the target velocity you calculated for the robot in previous task, and
	#	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
	#	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
	############################################
	clear()#comment or remove this

   
def main():
	global right_wheel_pub, left_wheel_pub, front_wheel_pub, vel_x, vel_y, vel_z, kp_x, kp_y, kp_theta, current_time, Helper_time, index, des_x, des_y, des_theta, flag
	global vel_1, vel_2, vel_3

	vel_1, vel_2, vel_3 = Wrench(), Wrench(), Wrench()
	rospy.init_node('controller_node')

	signal.signal(signal.SIGINT, signal_handler)

	# NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task
	#	Use the below given topics to generate motion for the robot.
	right_wheel_pub = rospy.Publisher('/right_wheel_force', Wrench, queue_size=10)
	front_wheel_pub = rospy.Publisher('/front_wheel_force', Wrench, queue_size=10)
	left_wheel_pub = rospy.Publisher('/left_wheel_force', Wrench, queue_size=10)

	rospy.Subscriber('detected_aruco',Pose2D,aruco_feedback_Cb)
	rospy.Subscriber('task2_goals',PoseArray,task2_goals_Cb)
	
	rate = rospy.Rate(100)

	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Make use of the logic you have developed in previous task to go-to-goal.
	#	-> Extend your logic to handle the feedback that is in terms of pixels.
	#	-> Tune your controller accordingly.
	# 	-> In this task you have to further implement (Inverse Kinematics!)
	#      find three omni-wheel velocities (v1, v2, v3) = left/right/center_wheel_force (assumption to simplify)
	#      given velocity of the chassis (Vx, Vy, W)
	#	   

		
	while not rospy.is_shutdown():
		# Getting time for the arrays
		current_time = time.time()
		sample_time  = 	   2
		# Find error (in x, y and 0) in global frame
		# the /odom topic is giving pose of the robot in global frame
		# the desired pose is declared above and defined by you in global frame
		# therefore calculate error in global frame
		# rospy.Subscriber('task1_goals', PoseArray, task1_goals_Cb)	
		# # Taking input for about 1s 
		# rospy.Subscriber('task1_goals', PoseArray, task1_goals_Cb)
		
		if (des_x - 0.01 <= hola_x <= 0.01 + des_x and des_y - 0.01 <= hola_y <= des_y + 0.01 and  des_theta - 0.005 <= hola_theta <= des_theta + 0.005):
			if( current_time - Helper_time >= sample_time ):
				if( 0 <= index < len(x_goals) and 0 <= index < len(y_goals) and 0 <= index < len(theta_goals)):
					des_x = x_goals[index]
					des_y = y_goals[index]
					des_theta = theta_goals[index]
					index += 1
					if(index == len(x_goals)):
						if(index == 0):
							rospy.Subscriber('task1_goals', PoseArray, task1_goals_Cb)
						else: 
							index = 0
							rospy.sleep(5)
		else:
			Helper_time = current_time


		err_x = des_x - hola_x 
		err_y = des_y - hola_y
		err_z = 0
		err_theta = des_theta - hola_theta

		# Publishing errors in order to identify the problem by plotting on the rqt

		# err_x_pub.publish(err_x)
		
		# err_y_pub.publish(err_y)
		
		# err_theta_pub.publish(err_theta)
		cleanup(err_x,err_y,err_z)
		# For debugging
		# print("errors:",errors)
		# This is probably the crux of Task 1, figure this out and rest should be fine.

		# Finally implement a P controller 
		# to react to the error with velocities in x, y and 0.
		vel_x = kp_x*errors[0]
		vel_y = kp_y*errors[1]
		vel_z = kp_theta*err_theta

		vel_1, vel_2, vel_3 = inverse_kinematics(vel_x, vel_y, vel_z)

		front_wheel_pub.publish(vel_1)
		right_wheel_pub.publish(vel_2)
		left_wheel_pub.publish(vel_3)

		# Modify the condition to Switch to Next goal (given position in pixels instead of meters)

		rate.sleep()

    ############################################

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass

