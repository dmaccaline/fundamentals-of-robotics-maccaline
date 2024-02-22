#!/usr/bin/env python3

"""
Daniel Maccaline
HW4 - Fundamentals of robotics
"""

import rospy
from turtlesim_helper.msg import UnitsLabelled

global pub

#callback for subscriber
def subscriberCallback(msg):
    global pub

    #get parameter, default to meter
    unit = rospy.get_param('unit', 'meter')

    pubMsg = msg


    if unit == 'smoot':
        #if unit is smoot, override pubMsg with values for smoot/update unit
        pubMsg.value = msg.value * 1.7018
        pubMsg.units = "Smoot"
    elif unit == 'feet':
        #if unit is feet, override pubMsg with values for feet/update unit
        pubMsg.value = msg.value * 3.2084
        pubMsg.units = "feet"

    #publish message
    pub.publish(pubMsg)

#main
if __name__ == '__main__':
    #init rospy

    rospy.init_node('Conversions', anonymous=True)

    rospy.loginfo("Test")

    #sleep so first message is not ropped
    rospy.sleep(1)
    #init publisher to turtle control topic
    pub = rospy.Publisher('/convertedDistance', UnitsLabelled, queue_size=10)
    sub = rospy.Subscriber("/distanceTraveled", UnitsLabelled, subscriberCallback)
    while not rospy.is_shutdown():
        rospy.spin()