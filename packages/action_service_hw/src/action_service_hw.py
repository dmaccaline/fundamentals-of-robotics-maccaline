#!/usr/bin/env python3

"""
Daniel Maccaline
HW10 - Fundamentals of robotics
"""

import rospy
import actionlib
import example_action_server.msg
from example_service.srv import Fibonacci




#main
if __name__ == '__main__':
    #init rospy
    rospy.init_node('simplePublisher', anonymous=True)

    serviceArg = Fibonacci()
    serviceArg.order = 3

    actionArg = example_action_server.msg.FibonacciGoal(order=3)
   # actionArg.order = 3


    rospy.loginfo("Waiting for service")
    rospy.wait_for_service("calc_fibonacci")
    rospy.loginfo("Service found")

    serviceProxy = rospy.ServiceProxy("calc_fibonacci", Fibonacci)

    startTime = rospy.get_time()
    res = serviceProxy(3)
    endTime = rospy.get_time()
    rospy.loginfo("Result from server: " + str(res))
    rospy.loginfo("Code continued executing and result recieved in " + str(endTime-startTime) + " seconds for argument of 3")

    actionClient = actionlib.SimpleActionClient('fibonacci', example_action_server.msg.FibonacciAction)
    rospy.loginfo("Waiting for action service")
    actionClient.wait_for_server()
    rospy.loginfo("Action service found")

    startTime = rospy.get_time()
    actionClient.send_goal(actionArg)
    endTime = rospy.get_time()
    actionClient.wait_for_result()
    resultTime = rospy.get_time()

    rospy.loginfo("Result from Action Server: " + str(actionClient.get_result()))
    rospy.loginfo("Returned to code in " + str(endTime- startTime) + " seconds")
    rospy.loginfo("Got result in " + str(resultTime - startTime) + " seconds for an argument of 3")


    startTime = rospy.get_time()
    res = serviceProxy(5)
    endTime = rospy.get_time()
    rospy.loginfo("Result from server: " + str(res))
    rospy.loginfo("Code continued executing and result recieved in " + str(endTime-startTime) + " seconds for an argument of 5")

    actionClient = actionlib.SimpleActionClient('fibonacci', example_action_server.msg.FibonacciAction)
    rospy.loginfo("Waiting for action service")
    actionClient.wait_for_server()
    rospy.loginfo("Action service found")

    actionArg2 = example_action_server.msg.FibonacciGoal(order=5)
    startTime = rospy.get_time()
    actionClient.send_goal(actionArg2)
    endTime = rospy.get_time()
    actionClient.wait_for_result()
    resultTime = rospy.get_time()

    rospy.loginfo("Result from Action Server: " + str(actionClient.get_result()))
    rospy.loginfo("Returned to code in " + str(endTime- startTime) + " seconds")
    rospy.loginfo("Got result in " + str(resultTime - startTime) + " seconds for an argument of 5")








