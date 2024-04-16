#!/usr/bin/env python3

"""
Daniel Maccaline
lab2 - Fundamentals of robotics
"""
import rospy
from duckietown_msgs.msg import WheelsCmdStamped
from duckietown_msgs.msg import FSMState
from std_msgs.msg import String
global move
global debugPub

def subscriberCallback(msg):
    global move, debugPub
    debugPub.publish("Recieved new mode of: " + str(msg.state))
    if msg.state == "LANE_FOLLOWING":
        move = True
    else:
        move = False

#main
if __name__ == '__main__':
    global move, debugPub
    #init rospy
    rospy.init_node('lab1', anonymous=True)

    move = False
    state = 0

    turnCommand = WheelsCmdStamped()
    turnCommand.vel_left = 0.2
    turnCommand.vel_right = 0.1

    moveStraight = WheelsCmdStamped()
    moveStraight.vel_left = .2
    moveStraight.vel_right = .2

    commandStop = WheelsCmdStamped()
    commandStop.vel_left = 0
    commandStop.vel_right = 0

    currentMsg = commandStop

    #init publisher to turtle control topic
    debugPub = rospy.Publisher('/danietown/debugOut', String, queue_size=10)
    pub = rospy.Publisher('/danietown/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
    sub = rospy.Subscriber("/danietown/fsm_node/mode", FSMState, subscriberCallback)

    timeRemaining = 0
    while not rospy.is_shutdown():
        rospy.sleep(.1)
        if move:
            debugPub.publish("On State: " + str(state) + " With " + str(timeRemaining) + " seconds remaining")
            pub.publish(currentMsg)
            timeRemaining = timeRemaining -.1
            #if time for this state has elapsed, set next state
            if(timeRemaining <= 0):
                if(state == 0):
                    state = 1
                    currentMsg = moveStraight
                    timeRemaining = 2
                elif state == 2:
                    state = 2
                    currentMsg = commandStop
                    timeRemaining = 1
                elif state == 3:
                    state = 4
                    currentMsg = turnCommand
                    timeRemaining = 1.5
                else:
                    state = 0
                    currentMsg = commandStop
                    timeRemaining = 1
