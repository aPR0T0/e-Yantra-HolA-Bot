#!/usr/bin/env python3
import queue
import rospy
import sys
import traceback
# importing time to get input as an array and then execute each coordinates one by one
import time

# Adding Auto-Evaluator
from geometry_msgs.msg import PoseArray
# publishing to /cmd_vel with msg type: Twist
from geometry_msgs.msg import Twist
# subscribing to /odom with msg type: Odometry
from nav_msgs.msg import Odometry

# for finding sin() cos() 
import math
# for array operations
import numpy as np

# Getting the float64MultiArray for subscription
from std_msgs.msg import Float64MultiArray
# Odometry is given as a quaternion, but for the controller we'll need to find the orientaion 0 by converting to euler angle
from tf.transformations import euler_from_quaternion

# Initializing the variables
hola_x = 0
hola_y = 0
hola_theta = 0
vel_x = 0
vel_y = 0
vel_z = 0
Helper_time = 0
index = 0		# this will help us determine what element are we currently on

# Initialise variables that may be needed for the control loop
# For ex: x_d, y_d, theta_d (in **meters** and **radians**) for defining desired goal-pose.
des_x = 0
des_y = 0
des_theta = 0

# desired coordinates but in the form of the arrays
x_goals = []
y_goals = []
theta_goals = []

# and also Kp values for the P Controller
kp_x = 2
kp_y = 2
kp_theta = 5 #initializing kp

#Taking input from the user for the desired co-ordinates
# des_x, des_y, des_theta = map(float, input("Specify your desired x and y coordinate and also specify the orientation:").split())

# Taking input from a topic known as auto-eval
def task1_goals_Cb(msg):
	global x_goals, y_goals, theta_goals

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

# Function to recieve values from the user
def SetValue(msg):
		global kp_x, kp_y, kp_theta
		kp_x = msg.data[0]
		kp_y = msg.data[1]
		kp_theta = msg.data[2]

# Subscriber for the current orientation and the position
def odometryCb(msg):
		global hola_x, hola_y, hola_theta, orientation

		hola_x = round(msg.pose.pose.position.x,2)
		hola_y = round(msg.pose.pose.position.y,2)
		#the data recieved from the sensor in in quaternion form
		orientation = [ msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
		#So, we need to convert that data from quaternion to euler using an in-built function
		hola_theta = euler_from_quaternion(orientation)[2]

	# Write your code to take the msg and update the three variables

def main():
		global pub, vel_x, vel_y,vel_z, kp_x, kp_y, kp_theta, rotation_matrix, current_time, Helper_time, index, des_x, des_y, des_theta
		# Initialze Node
		# We'll leave this for you to figure out the syntax for 
		# initialising node named "controller"
		rospy.init_node('controller_node', anonymous=False)
		# Initialze Publisher and Subscriber
		pub = rospy.Publisher("/cmd_vel", Twist, queue_size=100)
		# We'll leave this for you to figure out the syntax for
		# initialising publisher and subscriber of cmd_vel and odom respectively
		# Declare a Twist message
		vel = Twist()
		# Initialise the required variables to 0
		# <This is explained below>
		
		# For maintaining control loop rate.
		rate = rospy.Rate(100)

		x_goals = [1, -1, -1, 1, 0]
		y_goals = [1, 1, -1, -1, 0]
		theta_goals = [math.pi, 3*math.pi/4, -3*math.pi/4, -math.pi/4, 0]
		#
		# For tuning, accepting p_values from the message publisher 
		rospy.Subscriber("P_val", Float64MultiArray, SetValue)
		#
		#
		# declare that the node subscribes to task1_goals along with the other declarations of publishing and subscribing
		rospy.Subscriber('task1_goals', PoseArray, task1_goals_Cb)		


		while not rospy.is_shutdown():
			rospy.Subscriber("/odom", Odometry, odometryCb)

			# Getting time for the arrays
			current_time = time.time()
			sample_time = 2
			# Find error (in x, y and 0) in global frame
			# the /odom topic is giving pose of the robot in global frame
			# the desired pose is declared above and defined by you in global frame
			# therefore calculate error in global frame
			
			# Taking input for about 1s 
			if (des_x - 0.01 <= hola_x <= 0.01 + des_x and des_y - 0.01 <= hola_y <= des_y + 0.01 and  des_theta - 0.005 <= hola_theta <= des_theta + 0.005):
				if( current_time - Helper_time >= sample_time ):
					if( 0 < index < len(x_goals)):
						des_x = x_goals[index]
						des_y = y_goals[index]
						des_theta = theta_goals[index]
						index += 1
						if(index>=len(x_goals)):
							print("------------------------------------------")
							print("          Python Script ended!!           ")
							print("------------------------------------------")
					else:
						des_x = x_goals[index]
						des_y = y_goals[index]
						des_theta = theta_goals[index]
						index += 1
						if(index>=len(x_goals)):
							print("------------------------------------------")
							print("          Python Script ended!!           ")
							print("------------------------------------------")
			else:
				Helper_time = current_time

			err_x = des_x - hola_x
			err_y = des_y - hola_y
			err_z = 0
			err_theta = des_theta - hola_theta
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
			# For debugging
			# print("errors:",errors)
			# This is probably the crux of Task 1, figure this out and rest should be fine.

			# Finally implement a P controller 
			# to react to the error with velocities in x, y and 0.
			vel_x = kp_x*errors[0]
			vel_y = kp_y*errors[1]
			vel_z = kp_theta*err_theta
			# Safety Check
			# make sure the velocities are within a range.
			# for now since we are in a simulator and we are not dealing with actual physical limits on the system 
			# we may get away with skipping this step. But it will be very necessary in the long run.

			vel.linear.x = vel_x
			vel.linear.y = vel_y
			vel.angular.z = vel_z

			pub.publish(vel)
			rate.sleep()

if __name__=='__main__':

	try:
		print("------------------------------------------")
		print("         Python Script Started!!          ")
		print("------------------------------------------")
		main()
	except rospy.ROSInterruptException:
		pass