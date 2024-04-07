#!/usr/bin/env python3

"""
Daniel Maccaline
HW8 - Fundamentals of robotics
"""
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

global pub, pubWhite, pubYellow
global latestCroppedImage
global edges


#From example assignment
def output_lines(original_image, lines):
    output = np.copy(original_image)
    if lines is not None:
        for i in range(len(lines)):
            l = lines[i][0]
            cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
            cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
            cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
    return output


#callback for main cropped image, performs edge detection and stores cropped and edges globally
def subscriberCallback(msg):
    global pub, latestCroppedImage, edges
    #Convert to cv image
    bridge = CvBridge()
    latestCroppedImage = bridge.imgmsg_to_cv2(msg, "bgr8")

    edges = cv2.Canny(latestCroppedImage, 200, 300)

    #convert to ros msg and publish
    rosImage = bridge.cv2_to_imgmsg(edges, "8UC1")
    pub.publish(rosImage)



def whiteLineCallback(msgW):
    global edges, latestCroppedImage, pubWhite
    #Return if edges not set
    if(edges is None):
        return
    #Convert to cv image
    bridge = CvBridge()
    cvImage = bridge.imgmsg_to_cv2(msgW, "8UC1")


    kernel = np.ones((9, 9), np.uint8)
    cvImage = cv2.dilate(cvImage, kernel, iterations=1)

#apply bitwise and between recieved msg and edges
    whiteEdges = cv2.bitwise_and(cvImage, cvImage, mask = edges)


    linesW = cv2.HoughLinesP(whiteEdges, 1, np.pi/180, 10)

    imgW = output_lines(latestCroppedImage, linesW)
    rosImageW = bridge.cv2_to_imgmsg(imgW, "bgr8")
    pubWhite.publish(rosImageW)

def yellowLineCallback(msgY):
    global edges, latestCroppedImage, pubYellow
    #Return if edges not set
    if(edges is None):
        return

    #Convert to cv image
    bridge = CvBridge()
    cvImage = bridge.imgmsg_to_cv2(msgY, "8UC1")

    kernel = np.ones((9, 9), np.uint8)
    cvImage = cv2.dilate(cvImage, kernel, iterations=1)

    #apply bitwise and between recieved msg and edges
    yellowEdges = cv2.bitwise_and(cvImage, cvImage, mask = edges)

    linesY = cv2.HoughLinesP(yellowEdges, 1, np.pi/180, 10)

    imgY = output_lines(latestCroppedImage, linesY)
    rosImageY = bridge.cv2_to_imgmsg(imgY, "bgr8")
    pubYellow.publish(rosImageY)

#main
if __name__ == '__main__':
    global pub, pubWhite, pubYellow

    #init rospy
    rospy.init_node('simplePublisher', anonymous=True)

    #init publisher to turtle control topic
    pub = rospy.Publisher('/image_edges', Image, queue_size=10)
    pubWhite = rospy.Publisher('/image_lines_white', Image, queue_size=10)
    pubYellow = rospy.Publisher('/image_lines_yellow', Image, queue_size=10)
    sub = rospy.Subscriber("/image_cropped", Image, subscriberCallback)
    subWhite = rospy.Subscriber("/image_white", Image, whiteLineCallback)
    subYellow = rospy.Subscriber("/image_yellow", Image, yellowLineCallback)

    while not rospy.is_shutdown():
        rospy.spin()

