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
import cv2	
from cv_bridge import CvBridge	# Package to convert between ROS and OpenCV Images			# OpenCV Library
import math				# If you find it required
from geometry_msgs.msg import Pose2D	# Required to publish ARUCO's detected position & orientation


############################ GLOBALS #############################

aruco_publisher = rospy.Publisher('detected_aruco', Pose2D)
aruco_msg = Pose2D()

##################### FUNCTION DEFINITIONS #######################

# NOTE :  You may define multiple helper functions here and use in your code
def isRotationMatrix(R) :
    Rt = numpy.transpose(R)
    shouldBeIdentity = numpy.dot(Rt, R)
    I = numpy.identity(3, dtype = R.dtype)
    n = numpy.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
    assert(isRotationMatrix(R))
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return numpy.array([x, y, z])

def callback(data):
    # Bridge is Used to Convert ROS Image message to OpenCV image
    br = CvBridge()
    camera_matrix =  numpy.array([[1171.5121418959693, 0.0, 640.5],[0.0, 1171.5121418959693, 640.5],[0.0, 0.0, 1.0]])
    rospy.loginfo("receiving camera frame")
    get_frame = br.imgmsg_to_cv2(data, desired_encoding='bgr8')		# Receiving raw image in a "grayscale" format
    current_frame = cv2.resize(get_frame, (500, 500), interpolation = cv2.INTER_LINEAR)
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
    arucoParams = cv2.aruco.DetectorParameters_create()
    corners, ids, rejected = cv2.aruco.detectMarkers(current_frame, arucoDict,parameters=arucoParams)
    # print(corners, ids, rejected)
    rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(corners,15,camera_matrix,numpy.array([0.0, 0.0, 0.0, 0.0, 0.0]))
    # print(tvecs, rvecs)

# Flipping the vectors since they provide transformation from world frame to camera frame, and we require the reverse.
    rotation_matrix, jacobian = cv2.Rodrigues(rvecs)
# Computing the rotational matrix
    rw_tvec = numpy.matmul(rotation_matrix, numpy.transpose(tvecs[0])) # Computing the real world translational vector.
    roll, pitch, yaw = rotationMatrixToEulerAngles(rotation_matrix) # Computing RPY
    aruco_msg.x = rw_tvec[0]  #Translation in X
    aruco_msg.y = rw_tvec[1]  #Translation in Y
    aruco_msg.theta = yaw     #Rotation in Z
    print(aruco_msg)
    aruco_publisher.publish(aruco_msg)	#Publishing a Pose2D Message     

    cv2.aruco.drawDetectedMarkers(current_frame,corners)
    for i in range(len(rvecs)):
        rvec = rvecs[0][i]
        tvec = tvecs[0][i]
        cv2.drawFrameAxes(current_frame, camera_matrix, numpy.array([0.0, 0.0, 0.0, 0.0, 0.0]), rvec, tvec, 15 )
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
	rospy.Subscriber('overhead_cam/image_raw', Image, callback)
	rospy.spin()
  
if __name__ == '__main__':
  main()