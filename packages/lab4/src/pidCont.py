#!/usr/bin/env python3

"""
Daniel Maccaline
lab4 - Fundamentals of robotics
adapted from PID
"""

import rospy
from std_msgs.msg import Float32


class Pid:

    """
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
    """

#modified callback, no longer called by subscriber but from master node
    def getControlInput(self, msg):
        self.currentAccel = self.kp * (msg)

        #take current time
        self.timeNow = rospy.get_time()

        #if first run, skip this step (no previous time)
        if self.timePrev != -1:
            #Calculate approximate integral, using current location and time since last measurement
            self.integral = self.integral + self.ki * msg * (self.timeNow - self.timePrev)

            deriv = self.kd * (msg - self.errorPrev) / (self.timeNow - self.timePrev)
            return(self.currentAccel + self.integral + deriv)
        else:
            return(self.currentAccel)

        #store current time/error for next iteration
        self.timePrev = self.timeNow
        self.errorPrev = msg

        return
        
        

    def desiredCallback(self, msg):

        self.goal.data = msg.data

        return
        
    def enableController(self):
        enabled = True
        print("PID Controller enabled")
        return
    
    def disableController(self):
        enabled = True
        print("PID Controller enabled")
        return
        

    enabled = False
    pub = None
    positionSub = None
    currentAccel = Float32(-5)
    integral = 0
    timePrev = -1
    timeNow = 0
    errorPrev = -1
    kp = .215
    #kp = .5
    ki = .01
    kd = .6
    goal = 0

    def __init__(self, p = 0, i = 0, d = 0):


        #self.pub = rospy.Publisher('/control_input', Float32, queue_size=10)
        self.kp = p
        self.ki = i
        self.kd = d


    def setGoal(self, goal):
        self.goal = goal



if __name__ == '__main__':

    rospy.init_node('PIDController', anonymous=True)


    pidCont = Pid(.215, .01, .6)
    
    while not rospy.is_shutdown():
        rospy.spin()
