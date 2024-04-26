#!/usr/bin/env python3

"""
Daniel Maccaline
lab4 - Fundamentals of robotics
Using pid controllers from pidCont.py
"""
from pidCont import Pid
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String
from duckietown_msgs.msg import LanePose
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import Twist2DStamped


#Pid controllers
global rotational, linear
#publisher to send msg to control input
global pub

#publisher to aid debug
global debugPub

#subscribers, stateSub listens to fsm_node to activate/deactivate code, posSub gets LanePose
global stateSub, posSub

#Tracks if fsm_node has activated the node
global active




def subscriberCallback(msg):
    global active
    if msg.state == "LANE_FOLLOWING":
        debugPub.publish("Lane Following")
        rospy.sleep(2)
        active = True
    else:
        active = False



def pidCallback(msg):
    global rotational, linear, pub,active
    
    command = Twist2DStamped()
    
    #if not activated, ignore all messages and return immediatly
    if not active:
        command.v = 0
        command.omega = 0
    
        pub.publish(command)
        return
    
        
    #if get negative value, lane to the left, so turn left, else turn right
    val = rotational.getControlInput(msg.d - msg.d_ref);
    
    #Desired is .2, if -, move forward
    valV = linear.getControlInput(msg.phi-msg.phi_ref);
    
    
    
    debugPub.publish("Turn Val: " + str(val) + "\nVeloc Val: " + str(valV))
    #expecting a value between 0 and .05
    #want to command between .05 -> .5
    command.omega = (val + valV *1.8 )*-10
    command.v = .25
    
    pub.publish(command)
   
        



if __name__ == '__main__':

    global rotational, linear, pub, stateSub, posSub, active
    

    rospy.init_node('GradController', anonymous=True)
    
    active = False


    pub = rospy.Publisher('/danietown/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=10)
    stateSub = rospy.Subscriber("/danietown/fsm_node/mode", FSMState, subscriberCallback, queue_size=1)
    posSub = rospy.Subscriber("/danietown/lane_filter_node/lane_pose", LanePose, pidCallback)
    debugPub = rospy.Publisher('/debugOut', String, queue_size=10)

    #rotational = Pid(.4, .01, .2)
    rotational = Pid(.5, .01, .2)
    rotational.setGoal(0.019)
    
    #linear = Pid(.4, .01, .1)
    linear = Pid(.4, .01, .1)
    linear.setGoal = .02

    while not rospy.is_shutdown():
        rospy.spin()
