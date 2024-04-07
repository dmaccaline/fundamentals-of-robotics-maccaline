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
    croppedHSV = cv2.GaussianBlur(croppedHSV,(13,13),0)


    whiteLines = cv2.inRange(croppedHSV, (20, 0, 180), (255, 30, 255))
    yellowLines = cv2.inRange(croppedHSV, (0, 138, 157), (128, 255, 255))

    #apply open then close to clean image
    whiteLines = cv2.morphologyEx(whiteLines, cv2.MORPH_OPEN, (5,5))
    whiteLines = cv2.morphologyEx(whiteLines, cv2.MORPH_CLOSE, (5,5))

    yellowLines = cv2.morphologyEx(yellowLines, cv2.MORPH_OPEN, (5,5))
    yellowLines = cv2.morphologyEx(yellowLines, cv2.MORPH_CLOSE, (5,5))

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

