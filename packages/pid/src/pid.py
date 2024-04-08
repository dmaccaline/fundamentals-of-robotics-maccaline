#!/usr/bin/env python3

"""
Daniel Maccaline
HW10 - Fundamentals of robotics
"""

import rospy
from std_msgs.msg import Float32


class Pid:

    def errorCallback(self, msg):

        return

    def posiitionCallback(self, msg):

        self.currentAccel = self.kp * (msg.data)

        #take current time
        self.timeNow = rospy.get_time()

        #if first run, skip this step (no previous time)
        if self.timePrev != -1:
            #Calculate approximate integral, using current location and time since last measurement
            self.integral = self.integral + self.ki * msg.data * (self.timeNow - self.timePrev)

            deriv = self.kd * (msg.data - self.errorPrev) / (self.timeNow - self.timePrev)
            self.pub.publish(self.currentAccel + self.integral + deriv)
        else:
            self.pub.publish(self.currentAccel)

        #store current time/error for next iteration
        self.timePrev = self.timeNow
        self.errorPrev = msg.data

        #calculate derivative


        return

    def desiredCallback(self, msg):

        self.goal.data = msg.data

        return

    pub = None
    errorSub = None
    velocitySub = None
    positionSub = None
    goal = Float32()
    currentAccel = Float32(-5)
    integral = 0
    timePrev = -1
    timeNow = 0
    errorPrev = -1
    kp = .215
    #kp = .5
    ki = .01
    kd = .6

    def __init__(self):


        self.pub = rospy.Publisher('/control_input', Float32, queue_size=10)
        self.errorSub = rospy.Subscriber("/error", Float32, self.errorCallback)
        self.positionSub = rospy.Subscriber("/error", Float32, self.posiitionCallback)
        self.velocitySub = rospy.Subscriber("/desired", Float32, self.desiredCallback)


    def setGoal(self, goal):
        self.goal.data = goal



if __name__ == '__main__':

    rospy.init_node('PIDController', anonymous=True)


    pidCont = Pid()

    pidCont.setGoal(30)

    rospy.set_param('controller_ready', "ready")

    while not rospy.is_shutdown():
        rospy.spin()