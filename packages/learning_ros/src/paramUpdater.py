#!/usr/bin/env python3

"""
Daniel Maccaline
HW4 - Fundamentals of robotics
"""

import rospy


#main
if __name__ == '__main__':
    #init rospy

    rospy.init_node('ParamUodater', anonymous=True)

    unitList = ['meter', 'feet', 'smoot']
    #rather than waiting 5 seconds, wait for 1 second 5 times
        #allows for the program to be exited/interrupted without having to wait for a long sleep to end
    item = 1
    seconds = 5

    while not rospy.is_shutdown():

        rospy.sleep(1)

        if seconds <= 0:
            seconds = 5
            rospy.set_param('unit', unitList[item-1])
            rospy.loginfo("Unit set to " + unitList[item-1])
            if item >= 3:
                item = 1
            else:
                item = item + 1
        else:
            seconds = seconds - 1


