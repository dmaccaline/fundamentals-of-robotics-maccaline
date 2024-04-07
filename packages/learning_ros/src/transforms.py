#!/usr/bin/env python3

"""
Daniel Maccaline
HW5 - Fundamentals of robotics
"""

import rospy
import numpy as np
from duckietown_msgs.msg import Vector2D
import math

#used for debug
global printInfo

#Trnaform Matrix (set in main)
global sensorToWorld
global sensorToRobot

global pubRobot, pubWorld

#callback for subscriber
def subscriberCallback(msg):
    global sensorToWorld, sensorToRobot, printInfo, pubRobot, pubWorld
    if(printInfo):
        print("Recieved point: (" + str(msg.x) + ", " + str(msg.y) + ") in Sensor frame")

    pointLocation = np.matrix([[msg.x], [msg.y], [1]])


    pointInRobot = Vector2D()
    pointInWorld = Vector2D()

    robotLocatiton = sensorToRobot*pointLocation
    pointInRobot.x = robotLocatiton[0,0]
    pointInRobot.y = robotLocatiton[1,0]
    worldLocation = sensorToWorld*pointLocation
    pointInWorld.x = worldLocation[0,0]
    pointInWorld.y = worldLocation[1,0]

    if(printInfo):
        print("Point in Robot:")
        print(robotLocatiton)

        print("Published Point: (" + str(pointInRobot.x) + ", " + str(pointInRobot.y) + ")")

        print("Point in World:")
        print(worldLocation)

        print("Published Point: (" + str(pointInWorld.x) + ", " + str(pointInWorld.y) + ")")

    pubRobot.publish(pointInRobot)
    pubWorld.publish(pointInWorld)


#main
if __name__ == '__main__':
    #init rospy
    rospy.init_node('simplePublisher', anonymous=True)

    global sensorToWorld, sensorToRobot, printInfo, pubRobot, pubWorld
    #used for debug
    printInfo = False

    # Set sensorToRobot (x = -1. y = 0, theta = 180 deg/pi rads)
    # cos = -1, sin = 0,
#    sensorToRobot = np.matrix([[np.cos(math.pi), -1*np.sin(math.pi), -1], [np.sin(math.pi), np.cos(math.pi), 0], [0, 0, 1]])
    sensorToRobot = np.matrix([[-1, 0, -1], [0, -1, 0], [0, 0, 1]])

    #set robot to World (x = 6, y = 2, theta = 60+90 = 150 deg/ (150 * pi/180) rads)
    #robotToWorld = np.matrix([[np.cos(150 * (math.pi/180)), -1*np.sin(150 * (math.pi/180)), 6], [np.sin(150 * (math.pi/180)), np.cos(150 * (math.pi/180)), 2], [0, 0, 1]])

    robotToWorld = np.matrix([[np.cos(2.61799), -1*np.sin(2.61799), 6], [np.sin(2.61799), np.cos(2.61799), 2], [0, 0, 1]])

#create sensorToWorld from sensorToRobot and robotToWorld
    sensorToWorld = robotToWorld * sensorToRobot

    if(printInfo):
        print("Sensor To Robot:")
        print(sensorToRobot)
        print("Robot To World:")
        print(robotToWorld)
        print("Sensor To World:")
        print(sensorToWorld)

#sleep so first message is not ropped
    rospy.sleep(1)
    #init publisher to turtle control topic
    pubRobot = rospy.Publisher('/robot_coord', Vector2D, queue_size=10)
    pubWorld = rospy.Publisher('/world_coord', Vector2D, queue_size=10)
    sub = rospy.Subscriber("/sensor_coord", Vector2D, subscriberCallback)

    print("Waiting for Vector2D points on /sensor_coord topic")
    while not rospy.is_shutdown():
        rospy.spin()