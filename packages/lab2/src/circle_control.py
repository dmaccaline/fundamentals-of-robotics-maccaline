#!/usr/bin/env python3

"""
Daniel Maccaline
lab1 - Fundamentals of robotics
"""
import rospy
from duckietown_msgs.msg import WheelsCmdStamped
from duckietown_msgs.msg import FSMState

global move

def subscriberCallback(msg):
    if msg.state == "LANE_FOLLOWING":
        move = True
    else:
        move = False

#main
if __name__ == '__main__':

    #init rospy
    rospy.init_node('lab1', anonymous=True)

    move = False

    turnCommand = WheelsCmdStamped()
    turnCommand.vel_left = 0.0
    turnCommand.vel_right = 0.1

    #init publisher to turtle control topic
    pub = rospy.Publisher('/danietown/wheels_river_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
    sub = rospy.Subscriber("/danietown/fsm_node/mode", FSMState, subscriberCallback)

    while not rospy.is_shutdown():
        if move:
            pub.publish()


