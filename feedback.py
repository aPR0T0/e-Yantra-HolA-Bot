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


######################## IMPORT MODULES ##########################

import numpy				# If you find it required
import rospy 				
from sensor_msgs.msg import Image 	# Image is the message type for images in ROS
from cv_bridge import CvBridge	# Package to convert between ROS and OpenCV Images
import cv2				# OpenCV Library
import math				# If you find it required
from geometry_msgs.msg import Pose2D	# Required to publish ARUCO's detected position & orientation

############################ GLOBALS #############################

aruco_publisher = rospy.Publisher('detected_aruco', Pose2D)
aruco_msg = Pose2D()

##################### FUNCTION DEFINITIONS #######################

# NOTE :  You may define multiple helper functions here and use in your code

def callback(data):
	# Bridge is Used to Convert ROS Image message to OpenCV image
	br = CvBridge()						# To convert between ROS Image Messages and an Image for OpenCV
	rospy.loginfo("receiving camera frame")
	get_frame = br.imgmsg_to_cv2(data, "mono8")		# Receiving raw image in a "grayscale" format
	current_frame = cv2.resize(get_frame, (500, 500), interpolation = cv2.INTER_LINEAR)	#Resizing the CV Image Frame

	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Use OpenCV to find ARUCO MARKER from the IMAGE
	#	-> You are allowed to use any other library for ARUCO detection, 
	#        but the code should be strictly written by your team and
	#	   your code should take image & publish coordinates on the topics as specified only.  
	#	-> Use basic high-school geometry of "TRAPEZOIDAL SHAPES" to find accurate marker coordinates & orientation :)
	#	-> Observe the accuracy of aruco detection & handle every possible corner cases to get maximum scores !

	############################################

        marker_size = 1					#Desribes the number of markers to be detected 
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)		#To obtained a pre-defined dictionary of Aruco Markers
	corners, ids, rejected = cv2.aruco.detectMarkers(current_frame, aruco_dict, camera_matrix, camera_distortion)
	# corners = Stores the corners of the detected ArUCo marker.
	# ids = Stores the IDs of the detected ArUCo marker.
	# camera_matrix & camera_distortion = Associated with camera callibration, to be discussed.

	if ids is not None:	# Check if the number of detected markers is non-zero
                aruco.drawDetectedMarkers(current_frame,corners) #Draw a green outline arounded the detected marker.
                rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
		# Estimating the translational and rotational vectors for transformation between world frame and camera frame.
                rvec_rev, tvec_rev = rvec * -1, tvec * -1
		# Flipping the vectors since they provide transformation from world frame to camera frame, and we require the reverse.
                rotation_matrix, jacobian = cv2.Rodrigues(rvec_rev)
		# Computing the rotational matrix
                rw_tvec = np.dot(rotation_matrix, tvec_rev) # Computing the real world translational vector.
                roll, pitch, yaw = rotationMatrixToEulerAngles(rotation_matrix) # Computing RPY
                aruco_msg.x = rw_tvec[0]  #Translation in X
                aruco_msg.y = rw_tvec[1]  #Translation in Y
                aruco_msg.theta = yaw     #Rotation in Z
                aruco_publisher.publish(aruco_msg)	#Publishing a Pose2D Message                
                

def main():
	rospy.init_node('aruco_feedback_node')				#Creating a node  
	rospy.Subscriber('overhead_cam/image_raw', Image, callback)	#Subscribing to Overhead Camera Topic
	rospy.spin()					
  
if __name__ == '__main__':
  main()
