#!/usr/bin/env python3

"""
Daniel Maccaline
lab2 - Fundamentals of robotics
"""
import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from std_msgs.msg import String
global move
global debugPub

def subscriberCallback(msg):
    global move, debugPub
    debugPub.publish("Recieved new mode of: " + str(msg.state))
    if msg.state == "LANE_FOLLOWING":
        rospy.sleep(2)
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

    turnCommand = Twist2DStamped()
    turnCommand.v = 0
    turnCommand.omega = -2.5

    moveStraight = Twist2DStamped()
    moveStraight.v = -.2
    moveStraight.omega = 0

    commandStop = Twist2DStamped()
    commandStop.v = 0
    commandStop.omega = 0

    currentMsg = commandStop

    #init publisher to turtle control topic
    debugPub = rospy.Publisher('/debugOut', String, queue_size=10)
    pub = rospy.Publisher('/danietown/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=10)
    sub = rospy.Subscriber("/danietown/fsm_node/mode", FSMState, subscriberCallback)

    legs = 0
    timeRemaining = 0
    while not rospy.is_shutdown():
        rospy.sleep(.01)
        if move:
            debugPub.publish("On State: " + str(state) + " With " + str(timeRemaining) + " seconds remaining")
            pub.publish(currentMsg)
            timeRemaining = timeRemaining -.01
            #if time for this state has elapsed, set next state
            if(timeRemaining <= 0):
                if(state == 0):
                    state = 1
                    currentMsg = moveStraight
                    timeRemaining = 4.7
                    if(legs == 4):
                        break
                    legs = legs + 1
                elif state == 1:
                    state = 2
                    currentMsg = commandStop
                    timeRemaining = 1
                elif state == 2:
                    state = 3
                    currentMsg = turnCommand
                    timeRemaining = 0.58
                else:
                    state = 0
                    currentMsg = commandStop
                    timeRemaining = 1

