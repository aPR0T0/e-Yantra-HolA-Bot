#!/usr/bin/env python3

'''
*******************************
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
*******************************
'''

# Team ID:		[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		feedback.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node


######################## IMPORT MODULES ##########################

import numpy				# If you find it required
import rospy 				
from sensor_msgs.msg import Image 	# Image is the message type for images in ROS
import cv2	
from cv_bridge import CvBridge	# Package to convert between ROS and OpenCV Images			# OpenCV Library
import math				# If you find it required
from geometry_msgs.msg import Pose2D	# Required to publish ARUCO's detected position & orientation
import requests
import json 


############################ GLOBALS #############################

#aruco_publisher = rospy.Publisher('detected_aruco', Pose2D)
#aruco_msg = Pose2D()

##################### FUNCTION DEFINITIONS #######################

# NOTE :  You may define multiple helper functions here and use in your code

def callback(data):
	# Bridge is Used to Convert ROS Image message to OpenCV image
	br = CvBridge()
	rospy.loginfo("receiving camera frame")
	get_frame = br.imgmsg_to_cv2(data, desired_encoding='bgr8')		# Receiving raw image in a "grayscale" format
	current_frame = cv2.resize(get_frame, (500, 500), interpolation = cv2.INTER_LINEAR)
	arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
	arucoParams = cv2.aruco.DetectorParameters_create()
	(corners, ids, rejected) = cv2.aruco.detectMarkers(current_frame, arucoDict,parameters=arucoParams)
	print(corners, ids, rejected)
	if len(corners) > 0:
        # flatten the ArUco IDs list
		ids = ids.flatten()
		for i in range(len(corners)):
			(topLeft, topRight, bottomRight, bottomLeft) = corners[i][0][0],corners[i][0][1],corners[i][0][2],corners[i][0][3]
			topRight = (int(topRight[0]), int(topRight[1]))
			bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
			bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
			topLeft = (int(topLeft[0]), int(topLeft[1]))
			cv2.line(current_frame, topLeft, topRight, (0, 255, 0), 2)
			cv2.line(current_frame, topRight, bottomRight, (0, 255, 0), 2)
			cv2.line(current_frame, bottomRight, bottomLeft, (0, 255, 0), 2)
			cv2.line(current_frame, bottomLeft, topLeft, (0, 255, 0), 2)
			cX = int((topLeft[0] + bottomRight[0]) / 2.0)
			cY = int((topLeft[1] + bottomRight[1]) / 2.0)
			cv2.circle(current_frame, (cX, cY), 4, (0, 0, 255), -1)
			rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(corners,0.1524,numpy.array([[339.629804, 0.000000, 265.833298],
	[0.000000, 335.356282, 155.561312],[0.000000, 0.000000, 1.000000]]),numpy.array([-0.254560, 0.034596, 0.007207, 0.025113, 0.000000]))
			print('Hi', rvecs, tvecs)

			# string_arr = numpy.array_str(rvecs)
			# pload = {"kp":string_arr,"ki":"jyfujfy","kd":"hghgf"}
			# print(json.dumps(pload))
			# r = requests.post('http://192.168.122.49/api/v1/pid', json.dumps(pload));
			# r_dictionary= r
			# print(r_dictionary)
		
	cv2.imshow("output", current_frame)
	cv2.waitKey(3)
	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Use OpenCV to find ARUCO MARKER from the IMAGE
	#	-> You are allowed to use any other library for ARUCO detection, 
	#        but the code should be strictly written by your team and
	#	   your code should take image & publish coordinates on the topics as specified only.  
	#	-> Use basic high-school geometry of "TRAPEZOIDAL SHAPES" to find accurate marker coordinates & orientation :)
	#	-> Observe the accuracy of aruco detection & handle every possible corner cases to get maximum scores !

	############################################
      
def main():
	rospy.init_node('aruco_feedback_node')  
	rospy.Subscriber('usb_cam/image_raw', Image, callback)
	#callback()
	rospy.spin()
  
if __name__ == '__main__':
  main()