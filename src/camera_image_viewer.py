#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


bridge = CvBridge()

low_h = 0
high_h = 255
low_s = 0
high_s = 255
low_v = 0
high_v = 255
low_gray = 0
high_gray = 255

def nothing(x):
	pass


def initialize():
	cv2.namedWindow("Camera Image", 1)
	cv2.namedWindow("Processed Image", 1)
	cv2.namedWindow("Binary Image", 1)

	cv2.namedWindow("Tuning", 1)

	cv2.createTrackbar("Low H", "Tuning", low_h, 179, nothing)
	cv2.createTrackbar("High H", "Tuning", high_h, 179, nothing)
	cv2.createTrackbar("Low S", "Tuning", low_s, 255, nothing)
	cv2.createTrackbar("High S", "Tuning", high_s, 255, nothing)
	cv2.createTrackbar("Low V", "Tuning", low_v, 255, nothing)
	cv2.createTrackbar("High V", "Tuning", high_v, 255, nothing)
	cv2.createTrackbar("Low Gray", "Tuning", low_gray, 255, nothing)
	cv2.createTrackbar("High Gray", "Tuning", high_gray, 255, nothing)


def displayImage(imageData):
	# Convert BGR8 to HSV
	hsvImage = cv2.cvtColor(imageData, cv2.COLOR_BGR2HSV)

	# Grab trackbar values
	low_h = cv2.getTrackbarPos("Low H", "Tuning")
	high_h = cv2.getTrackbarPos("High H", "Tuning")
	low_s = cv2.getTrackbarPos("Low S", "Tuning")
	high_s = cv2.getTrackbarPos("High S", "Tuning")
	low_v = cv2.getTrackbarPos("Low V", "Tuning")
	high_v = cv2.getTrackbarPos("High V", "Tuning")

	# define range of red color in HSV
	lower_red = np.array([low_h,low_s,low_v])
	upper_red = np.array([high_h,high_s,high_v])

	# Threshold the HSV image to get only red colors
	mask = cv2.inRange(hsvImage, lower_red, upper_red)

	# Bitwise-AND mask and original image
	result = cv2.bitwise_and(imageData, imageData, mask=mask)

	cv2.imshow("Processed Image", result)


	# Convert to grayscale
	grayImage = cv2.cvtColor(result, cv2.COLOR_HSV2BGR)
	grayImage = cv2.cvtColor(grayImage, cv2.COLOR_BGR2GRAY)	

	# Now pass the gray-scale image through a threshold to get a binary image.
	low_gray = cv2.getTrackbarPos("Low Gray", "Tuning")
	high_gray = cv2.getTrackbarPos("High Gray", "Tuning")
	#ret,binImage = cv2.threshold(grayImage, low_gray, high_gray, cv2.THRESH_BINARY)
	grayImage = cv2.GaussianBlur( grayImage, (5,5), 0 )
	binImage = cv2.adaptiveThreshold(grayImage, 155, 1, 1, 11, 2)
	
	#img = cv2.imread(binImage,0)
	#kernel = np.ones((5,5),np.uint8)
	#erosion = cv2.erode(binImage,kernel,iterations = 1)
	#dilation = cv2.dilate(img,kernel,iterations = 1)

	cv2.imshow("Binary Image", binImage)

	# Grab our contours and then draw an enclosing circle around the biggest one
	contours, _ = cv2.findContours( binImage, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
	if len(contours) > 0:
		biggestContour = contours[0]
		biggestContourArea = cv2.contourArea(biggestContour)

		for c in contours:
			currentContourArea = cv2.contourArea(c)
			if currentContourArea > biggestContourArea:
				biggestContour = c
				biggestContourArea = currentContourArea
		
		# Draw the original contour just so we know what we are dealing with.
		cv2.drawContours( imageData, np.array([biggestContour]), -1, (0,0,255), 2, cv2.CV_AA)

		# Draw the enclosing circle
		(x,y), radius = cv2.minEnclosingCircle(biggestContour)
		center = (int(x), int(y))
		radius = int(radius)
		cv2.circle( imageData, center, radius, (0,255,0), 2)

	cv2.imshow("Camera Image", imageData)


def detectCircles(grayImage):
	# TRACKBARS
	return cv2.HoughCircles(grayImage, cv2.cv.CV_HOUGH_GRADIENT, 1.2, 100)
	


def callback(data):
	cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
	displayImage(cv_image)
	cv2.waitKey(3)


def listener():	
	rospy.Subscriber("/usb_cam/image_raw", Image, callback)
	rospy.init_node("listener", anonymous=True)

	rospy.spin()

	cv2.destroyAllWindows()


if __name__ == '__main__':
	initialize()
	listener()

