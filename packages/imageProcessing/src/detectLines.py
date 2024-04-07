#!/usr/bin/env python3

"""
Daniel Maccaline
HW7 - Fundamentals of robotics
"""
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
global pub, pubWhite, pubYellow
from cv_bridge import CvBridge
import cv2
import numpy as np

#callback for subscriber
def subscriberCallback(msg):
    global pub, pubWhite, pubYellow
    #Convert to cv image
    bridge = CvBridge()
    cvImage = bridge.imgmsg_to_cv2(msg, "bgr8")

    #Get image dimensions
    dimensions = cvImage.shape
    #crop top half out
    cropped_image = cvImage[int(dimensions[0]/2):dimensions[0], 0:dimensions[1]]

    #convert to ros msg and publish
    rosImage = bridge.cv2_to_imgmsg(cropped_image, "bgr8")
    pub.publish(rosImage)

    croppedHSV = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)

    #applly gaussian blur
    croppedHSV = cv2.GaussianBlur(croppedHSV,(9,9),0)


    whiteLines = cv2.inRange(croppedHSV, (0, 0, 120), (255, 45, 255))
    yellowLines = cv2.inRange(croppedHSV, (0, 110, 125), (128, 255, 255))

    #apply erode/dilate to clean images
    kernel = np.ones((5, 5), np.uint8)
    whiteLines = cv2.dilate(whiteLines, kernel, iterations=1)
    whiteLines = cv2.erode(whiteLines, kernel, iterations=1)

    whiteLines = cv2.dilate(whiteLines, kernel, iterations=1)
    yellowLines = cv2.erode(yellowLines, kernel, iterations=1)

    #convert to ros msg and publish
    rosImage = bridge.cv2_to_imgmsg(whiteLines, "mono8")
    pubWhite.publish(rosImage)
    rosImage = bridge.cv2_to_imgmsg(yellowLines, "mono8")
    pubYellow.publish(rosImage)


#main
if __name__ == '__main__':
    global pub, pubWhite, pubYellow

    #init rospy
    rospy.init_node('simplePublisher', anonymous=True)

    #init publisher to turtle control topic
    pub = rospy.Publisher('/image_cropped', Image, queue_size=10)
    pubWhite = rospy.Publisher('/image_white', Image, queue_size=10)
    pubYellow = rospy.Publisher('/image_yellow', Image, queue_size=10)
    sub = rospy.Subscriber("/image", Image, subscriberCallback)

    while not rospy.is_shutdown():
        rospy.spin()

