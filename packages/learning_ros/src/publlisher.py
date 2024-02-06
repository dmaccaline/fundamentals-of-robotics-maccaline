#!/usr/bin/env python3

"""
Daniel Maccaline
HW2 - Fundamentals of robotics
"""

import rospy
from geometry_msgs.msg import Twist

#main
if __name__ == '__main__':
    #init rospy
    rospy.init_node('simplePublisher', anonymous=True)

    #sleep so first message is not ropped
    rospy.sleep(1)
    #init publisher to turtle control topic
    pub = rospy.Publisher('/turtlesim/turtle1/cmd_vel', Twist, queue_size=10)

    #crete two geometry_msg::twist variables to publish
    forwardMsg = Twist()
    forwardMsg.linear.x = 2     #Set x to 2 (move forward)
    TurnMsg = Twist()
    TurnMsg.angular.z = 1.57079632679   #turn 90 deg (1.57 radians)

    #loop until stopped
    while not rospy.is_shutdown():

        #publish forward msg
        pub.publish(forwardMsg)
        #give time for robot to complete movement
        rospy.sleep(1.8)
        #publish turn message
        pub.publish(TurnMsg)
        #give time for robot to complete movement
        rospy.sleep(1.2)

