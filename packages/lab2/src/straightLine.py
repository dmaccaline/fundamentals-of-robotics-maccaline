#!/usr/bin/env python3

"""
Daniel Maccaline
lab2 - Fundamentals of robotics
"""
import rospy
from duckietown_msgs.msg import WheelsCmdStamped
from duckietown_msgs.msg import FSMState

global move

def subscriberCallback(msg):
    global move
    if msg.state == "LANE_FOLLOWING":
        move = True
    else:
        move = False

#main
if __name__ == '__main__':

    #init rospy
    rospy.init_node('lab2_line', anonymous=True)

    move = False

    moveCommand = WheelsCmdStamped()
    moveCommand.vel_left = 0.3
    moveCommand.vel_right = 0.3

    stopCommand = WheelsCmdStamped()
    stopCommand.vel_left = 0
    stopCommand.vel_right = 0

    #init publisher to turtle control topic
    pub = rospy.Publisher('/danietown/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
    sub = rospy.Subscriber("/danietown/fsm_node/mode", FSMState, subscriberCallback)

    while not rospy.is_shutdown():
        if move:
            pub.publish(moveCommand)
            rospy.sleep(5)
            pub.publish(stopCommand)
            break


