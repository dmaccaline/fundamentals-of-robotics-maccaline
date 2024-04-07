#!/usr/bin/env python3

"""
Daniel Maccaline
HW6 - Fundamentals of robotics
"""
import rospy
from std_msgs.msg import String
from odometry_hw.msg import DistWheel
from odometry_hw.msg import Pose2D
import numpy as np

global pub
global position
global printDebug

#callback for subscriber
def subscriberCallback(msg):
    global pub, counter, position, printDebug

    #print debug if flag is true
    if(printDebug):
        print("Position X: " + str(position.x))
        print("Position Y: " + str(position.y))
        print("Theta: " + str(position.theta))


    #Calculate deltaS/ change in rotation
    deltaS = (msg.dist_wheel_left + msg.dist_wheel_right)/2
    deltaTheta = (msg.dist_wheel_left - msg.dist_wheel_right)/(.1)

    #Calculate change in position
    deltaX = deltaS * np.cos(position.theta + deltaTheta/2)
    deltaY = deltaS * np.sin(position.theta + deltaTheta/2)

    #Update position
    position.x = position.x + deltaX
    position.y = position.y + deltaY
    position.theta = position.theta + deltaTheta

    #Publish new position
    pub.publish(position)



#main
if __name__ == '__main__':
    global pub, counter, posititon, printDebug

    #Setup initial Position
    position = Pose2D()
    position.x = 0
    position.y = 2
    position.theta = 0

    #Set dubug flag
    printDebug = False

    #init rospy
    rospy.init_node('simplePublisher', anonymous=True)

    #init publisher to turtle control topic
    pub = rospy.Publisher('/pose', Pose2D, queue_size=10)
    sub = rospy.Subscriber("/dist_wheel", DistWheel, subscriberCallback)

    #Sleep to ensure other processes are running
    rospy.sleep(2)
    #Set param to ready
    rospy.set_param('odom_ready', "ready")
    while not rospy.is_shutdown():
        rospy.spin()

