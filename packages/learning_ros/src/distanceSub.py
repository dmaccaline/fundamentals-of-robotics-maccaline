#!/usr/bin/env python3

"""
Daniel Maccaline
HW2 - Fundamentals of robotics
"""

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import String
from numpy import sqrt
from turtlesim_helper.msg import UnitsLabelled

global prevMsg
global distance
global pub
global run

#callback for subscriber
def subscriberCallback(msg):
    global prevMsg, distance, pub, run

    if run == False:
        prevMsg = msg
        run = True

    else:
        #calculate distance
#        rospy.loginfo("X: " + str(msg.x) + " Prev X: " + str(prevMsg.x))
#        rospy.loginfo("Y: " + str(msg.y) + " Prev Y: " + str(prevMsg.y))
        x = msg.x - prevMsg.x
        y = msg.y - prevMsg.y

        x = x*x
        y = y*y
        sum = x+y
        distance = distance + sqrt(sum)

        pubMsg = UnitsLabelled()
        pubMsg.value = distance
        pubMsg.units = "Meters"
#        strMsg = "Distance traveleds " + str(distance) + " meters:"
#        rospy.loginfo(strMsg)
        pub.publish(pubMsg)
        prevMsg = msg


#main
if __name__ == '__main__':
    #init rospy
    global prevMsg, distance, pub, run
    prevMsg = Pose()
    distance = 0
    run = False

    rospy.init_node('simplePublisher', anonymous=True)

    #sleep so first message is not ropped
    rospy.sleep(1)
    #init publisher to turtle control topic
    pub = rospy.Publisher('/distanceTraveled', UnitsLabelled, queue_size=10)
    sub = rospy.Subscriber("/turtlesim/turtle1/pose", Pose, subscriberCallback)
    while not rospy.is_shutdown():
        rospy.spin()
