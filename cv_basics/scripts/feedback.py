#!/usr/bin/env python3
import numpy				# If you find it required
import rospy
from sensor_msgs.msg import Image 	# Image is the message type for images in ROS
import cv2
# Package to convert between ROS and OpenCV Images			# OpenCV Library
from cv_bridge import CvBridge
import math				# If you find it required
# Required to publish ARUCO's detected position & orientation
from geometry_msgs.msg import Pose2D
import requests
import json

count = 0
############################ GLOBALS #############################

# aruco_publisher = rospy.Publisher('detected_aruco', Pose2D)
# aruco_msg = Pose2D()

##################### FUNCTION DEFINITIONS #######################

# NOTE :  You may define multiple helper functions here and use in your code
cor_x = 0
cor_y = 0


def callback(data):
    # Bridge is Used to Convert ROS Image message to OpenCV image
    br = CvBridge()
    # rospy.loginfo("receiving camera frame")
    # Receiving raw image in a "grayscale" format
    get_frame = br.imgmsg_to_cv2(data, desired_encoding='bgr8')
    current_frame = cv2.resize(
        get_frame, (500, 500), interpolation=cv2.INTER_LINEAR)
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
    arucoParams = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(
        current_frame, arucoDict, parameters=arucoParams)
    # print(corners, ids, rejected)
    if len(corners) > 0:
        # flatten the ArUco IDs list
        ids = ids.flatten()
        print(ids)
        if ((0 in ids) and (1 in ids) and (2 in ids) and (3 in ids)):
            # print(corners[numpy.where(ids == 0)[0][0]][0][0])
            h, w = current_frame.shape[:2]
            dst_points = numpy.array(
                [[0, 0], [w, 0], [0, h], [w, h]], dtype=numpy.float32)
            src_points = numpy.array([corners[numpy.where(ids == 0)[0][0]][0][0] - 10, corners[numpy.where(
                ids == 1)[0][0]][0][1], corners[numpy.where(ids == 2)[0][0]][0][3], corners[numpy.where(ids == 3)[0][0]][0][2]], dtype=numpy.float32)
            M = cv2.getPerspectiveTransform(src_points, dst_points)
            img_straight = cv2.warpPerspective(current_frame, M, (w, h))
            img_straight = cv2.resize(img_straight, (500, 500))
            (corners, ids, rejected) = cv2.aruco.detectMarkers(
                img_straight, arucoDict, parameters=arucoParams)
            if len(corners) > 0:
                # flatten the ArUco IDs list
                ids = ids.flatten()
                if (4 in ids):

                    x = corners[numpy.where(ids == 4)[0][0]][0][0][0]
                    y = corners[numpy.where(ids == 4)[0][0]][0][0][1]
                    (topLeft_4, topRight_4, bottomRight_4, bottomLeft_4) = corners[numpy.where(ids == 4)[0][0]][0][0], corners[numpy.where(
                        ids == 4)[0][0]][0][1], corners[numpy.where(ids == 4)[0][0]][0][2], corners[numpy.where(ids == 4)[0][0]][0][3]
                    topRight_4 = (int(topRight_4[0]), int(topRight_4[1]))
                    bottomRight_4 = (
                        int(bottomRight_4[0]), int(bottomRight_4[1]))
                    bottomLeft_4 = (int(bottomLeft_4[0]), int(bottomLeft_4[1]))
                    topLeft_4 = (int(topLeft_4[0]), int(topLeft_4[1]))
                    orientation = math.atan2(-(topLeft_4[0] -
                                               topRight_4[0]), -(topLeft_4[1]-topRight_4[1]))
                    print("new : ", x, y, orientation)
                    try:
                        pload = {"kp": float(x), "ki": float(
                            y), "kd": float(orientation)}
                        print(json.dumps(pload))
                        r = requests.post(
                            'http://192.168.19.49/api/v1/pid', json.dumps(pload))
                        r_dictionary = r
                        print(r_dictionary)
                    except:
                        print("localhost not connected")
                    cv2.putText(img=img_straight, text=str("x : " + str(x) + "  y : " + str(y) + "   o : " + str(round(orientation, 4))),
                                org=(10, 30), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.6, color=(0, 255, 0), thickness=2)
                cv2.imshow("straight", img_straight)
                cv2.waitKey(3)

    if len(corners) > 0:
        # flatten the ArUco IDs list
        ids = ids.flatten()

        # print(ids)

        if (((0 in ids)) and (1 in ids) and (2 in ids) and (4 in ids)):
            for i in range(4):

                (topLeft, topRight, bottomRight,
                 bottomLeft) = corners[i][0][0], corners[i][0][1], corners[i][0][2], corners[i][0][3]

                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                cv2.line(current_frame, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(current_frame, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(current_frame, bottomRight,
                         bottomLeft, (0, 255, 0), 2)
                cv2.line(current_frame, bottomLeft, topLeft, (0, 255, 0), 2)

                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)

                cv2.circle(current_frame, (cX, cY), 4, (0, 0, 255), -1)
    ############ ADD YOUR CODE HERE ############

    # INSTRUCTIONS & HELP :
    # -> Use OpenCV to find ARUCO MARKER from the IMAGE
    # -> You are allowed to use any other library for ARUCO detection,
    #        but the code should be strictly written by your team and
    # your code should take image & publish coordinates on the topics as specified only.
    # -> Use basic high-school geometry of "TRAPEZOIDAL SHAPES" to find accurate marker coordinates & orientation :)
    # -> Observe the accuracy of aruco detection & handle every possible corner cases to get maximum scores !

    ############################################


def main():
    rospy.init_node('aruco_feedback_node')
    rospy.Subscriber('usb_cam/image_raw', Image, callback)
    rospy.spin()


if __name__ == '__main__':
    main()
