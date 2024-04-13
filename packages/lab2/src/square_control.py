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
    onStraight = True

    turnCommand = WheelsCmdStamped()
    turnCommand.vel_left = 0.2
    turnCommand.vel_right = 0.1

    moveStraight = WheelsCmdStamped()
    moveStraight.vel_left = .2
    moveStraight.vel_right = .2

    commandStop = WheelsCmdStamped()
    commandStop.vel_left = 0
    commandStop.vel_right = 0

    #init publisher to turtle control topic
    pub = rospy.Publisher('/danietown/wheels_river_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
    sub = rospy.Subscriber("/danietown/fsm_node/mode", FSMState, subscriberCallback)

    timeRemaining = 0
    while not rospy.is_shutdown():
        if move:
            if(timeRemaining > 0):
                rospy.sleep(.1)
            else:
                if(onStraight):
                    pub.publish(moveStraight)
                    timeRemaining = 2
                else:
                    pub.publish(turnCommand)
                    timeRemaining = 2

        rospy.spinOnce()

