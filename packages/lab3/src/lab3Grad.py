#!/usr/bin/env python3

"""
Daniel Maccaline
lab3 - Fundamentals of robotics (adapted from hw7/8)
"""
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage as Image
from sensor_msgs.msg import Image as ImageO
from duckietown_msgs.msg import SegmentList
from duckietown_msgs.msg import Segment
global pub, pubWhite, pubYellow, pubSegments, srv1, srv2
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_srvs.srv import SetBool, SetBoolResponse


def ld_switch(msg):
    return True, ""

def lf_switch(msg):
    return True, ""
    
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

def output_lines_color(original_image, lines, circleColor, lineColor):
    output = np.copy(original_image)
    if lines is not None:
        for i in range(len(lines)):
            l = lines[i][0]
            cv2.line(output, (l[0],l[1]), (l[2],l[3]), lineColor, 2, cv2.LINE_AA)
            cv2.circle(output, (l[0],l[1]), 2, circleColor)
            cv2.circle(output, (l[2],l[3]), 2, circleColor)
    return output



#callback for subscriber
def subscriberCallback(msg):
    global pub, pubWhite, pubYellow, pubSegments
    
    #Convert to cv image
    bridge = CvBridge()
    cvImage = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

    #Get image dimensions
    dimensions = cvImage.shape
    
    image_size = (160, 120)
    offset = 40
    new_image = cv2.resize(cvImage, image_size, interpolation=cv2.INTER_NEAREST)
    cropped_image = new_image[offset:, :]
    #crop top half out
    #cropped_image = cvImage[int(dimensions[0]/2):dimensions[0], 0:dimensions[1]]

    croppedHSV = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)

    #applly gaussian blur
    croppedHSV = cv2.GaussianBlur(croppedHSV,(9,9),0)


    whiteLines = cv2.inRange(croppedHSV, (0, 0, 170), (255, 55, 255))
    #yellowLines = cv2.inRange(croppedHSV, (0, 20, 100), (50, 140, 255))
    yellowLines = cv2.inRange(croppedHSV, (0, 20, 110), (50, 120, 255))
    


    #apply erode/dilate to clean images
    kernel = np.ones((5, 5), np.uint8)
    whiteLines = cv2.dilate(whiteLines, kernel, iterations=1)
    whiteLines = cv2.erode(whiteLines, kernel, iterations=1)

    whiteLines = cv2.dilate(whiteLines, kernel, iterations=1)
    yellowLines = cv2.erode(yellowLines, kernel, iterations=1)

    #Get edges from original image
    edges = cv2.Canny(cropped_image, 200, 300)

    #Draw while lines
    kernel = np.ones((11, 11), np.uint8)
    whiteLines = cv2.dilate(whiteLines, kernel, iterations=1)

    #apply bitwise and between recieved msg and edges
    whiteEdges = cv2.bitwise_and(whiteLines, whiteLines, mask = edges)

    linesW = cv2.HoughLinesP(whiteEdges, 1, np.pi/180, 10, minLineLength=1)

    imgW = output_lines(cropped_image, linesW)
    rosImageW = bridge.cv2_to_imgmsg(imgW, "bgr8")
    pubWhite.publish(rosImageW)

    #Draw while lines
    kernel = np.ones((9, 9), np.uint8)
    yellowLines = cv2.dilate(yellowLines, kernel, iterations=1)

    #apply bitwise and between recieved msg and edges
    yellowEdges = cv2.bitwise_and(yellowLines, yellowLines, mask = edges)

    linesY = cv2.HoughLinesP(yellowEdges, 1, np.pi/180, 10, minLineLength=1)

    imgW = output_lines(cropped_image, linesY)
    rosImageY = bridge.cv2_to_imgmsg(imgW, "bgr8")
    pubYellow.publish(rosImageY)


    #pubish image with both lines
    imgCI = output_lines_color(cropped_image, linesW, (0, 255, 0), (255, 0, 0))
    imgC = output_lines_color(imgCI, linesY, (0, 255, 0), (0, 0, 255))
    rosImageY = bridge.cv2_to_imgmsg(imgC, "bgr8")
    pub.publish(rosImageY)
    
    arr_cutoff = np.array([0, offset, 0, offset])
    arr_ratio = np.array([1. / image_size[0], 1. / image_size[1], 1. / image_size[0], 1. / image_size[1]])
    

    segList = []
    if linesW is not None:
        for i in range(len(linesW)):
            l = (linesW[i][0] + arr_cutoff) * arr_ratio
            tempSeg = Segment()
            tempSeg.color = 0
            tempSeg.pixels_normalized[0].x = (l[0])
            tempSeg.pixels_normalized[0].y = (l[1])
            tempSeg.pixels_normalized[1].x = (l[2])
            tempSeg.pixels_normalized[1].y = (l[3])
            segList.append(tempSeg)
            

    if linesY is not None:
        for i in range(len(linesY)):
            l = (linesY[i][0] + arr_cutoff) * arr_ratio
            tempSeg = Segment()
            tempSeg.color = 1
            tempSeg.pixels_normalized[0].x = (l[0])
            tempSeg.pixels_normalized[0].y = (l[1])
            tempSeg.pixels_normalized[1].x = (l[2])
            tempSeg.pixels_normalized[1].y = (l[3])
            segList.append(tempSeg)
            
    linesTotal = SegmentList(segments=np.copy(segList))
    pubSegments.publish(linesTotal)




#main
if __name__ == '__main__':
    global pub, pubWhite, pubYellow, pubSegments, srv1, srv2

    #init rospy
    rospy.init_node('line_detect', anonymous=True)

    #init publisher to turtle control topic
    pub = rospy.Publisher('/danietown/allLines', ImageO, queue_size=10)
    pubWhite = rospy.Publisher('/danietown/whiteLines', ImageO, queue_size=10)
    pubYellow = rospy.Publisher('/danietown/yellowLines', ImageO, queue_size=10)
    pubSegments = rospy.Publisher('/danietown/line_detector_node/segment_list', SegmentList, queue_size=10)
    #sub = rospy.Subscriber("/image", Image, subscriberCallback, queue_size=1, buff_size=2**24)
    sub = rospy.Subscriber("/danietown/camera_node/image/compressed", Image, subscriberCallback, queue_size=1, buff_size = 2**24)
    
    srv1 = rospy.Service('line_detector_node/switch', SetBool, ld_switch)
    srv2 = rospy.Service('lane_filter_node/switch', SetBool, lf_switch)
    
    
    while not rospy.is_shutdown():
        rospy.spin()
