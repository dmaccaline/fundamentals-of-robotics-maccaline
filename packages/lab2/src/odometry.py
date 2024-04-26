#!/usr/bin/env python3

"""
Daniel Maccaline
lab2 - Fundamentals of robotics
"""
import rospy
from duckietown_msgs.msg import Twist2DStamped
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
import numpy as np


global pub, debugPub
global pos
global prevTime

def subscriberCallback(msg):
    global pub, pos, prevTime
    
    
    currentTime = rospy.get_time()
    if(prevTime == -1):
        prevTime = currentTime
        return
    
    #Calculate foward distance
    forwardDist = msg.v * (currentTime-prevTime)
    
    #calculate deltaOmega
    deltaOmega = msg.omega * (currentTime-prevTime)
    
    deltaX = forwardDist * np.cos(pos.theta + deltaOmega/2)
    deltaY = forwardDist * np.sin(pos.theta + deltaOmega/2)
    
    pos.x = pos.x + deltaX
    pos.y = pos.y + deltaY
    pos.theta = pos.theta + deltaOmega
    
    debugPub.publish("Time: " + str(currentTime-prevTime))
    prevTime = currentTime

    #Publish new position
    pub.publish(pos)
    
    

#main
if __name__ == '__main__':

    global pub, pos, prevTime, debugPub
    pos = Pose2D()
    pos.x = 0
    pos.y = 0
    pos.theta = 0
    prevTime = -1
    #init rospy
    rospy.init_node('lab1', anonymous=True)

    move = False

    
    #init publisher to turtle control topic
    pub = rospy.Publisher('/odometry_output', Pose2D, queue_size=10)
    debugPub = rospy.Publisher('/debugOut', String, queue_size=10)
    sub = rospy.Subscriber("/danietown/kinematics_node/velocity", Twist2DStamped, subscriberCallback)

    while not rospy.is_shutdown():
        rospy.spin()

