#!/usr/bin/python 

'''
----------------------------
Author: Avishai Sintov
        Rutgers University
Date: October 2018
----------------------------
'''


import rospy
import numpy as np 
from std_msgs.msg import Char
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from openhand.srv import MoveServos
from std_srvs.srv import Empty, EmptyResponse
# import tty, termios, sys
from std_srvs.srv import Empty, EmptyResponse
from hand_control.srv import observation, IsDropped, TargetAngles, RegraspObject, close, MovePrim
import curses, time

def key_listener(stdscr):
    """checking for keypress"""
    stdscr.nodelay(True)  # do not wait for input when calling getch
    return stdscr.getch()

class keyboard_control():

    def __init__(self):
        rospy.init_node('keyboard_control', anonymous=True)

        pub_action = rospy.Publisher('/keyboard/desired_action', Float32MultiArray, queue_size=10)
        rospy.Subscriber('/gripper/pos', Float32MultiArray, self.ActPosCallback)
        rospy.Service('/ResetKeyboard', Empty, self.ResetKeyboard)
        move_srv = rospy.ServiceProxy('/MovePrimitives', MovePrim)
        moveKey_srv = rospy.ServiceProxy('/MoveKeys', MovePrim)

        rate = rospy.Rate(5)
        self.ch = 's'
        while not rospy.is_shutdown():
            c = curses.wrapper(key_listener)#self.getchar()
            if c!=-1:
                self.ch = chr(c)

            if ord(self.ch) == 27:
                break

            if np.any(self.ch == np.array(['o','p','n','m','u','j'])):
                moveKey_srv(self.ch)
                self.ch = 's'
                rospy.sleep(1.0)
            else:
                move_srv(self.ch)

            rate.sleep()

    def ActPosCallback(self, msg):
        self.act_angles = np.array(msg.data)

    def ResetKeyboard(self, msg):
        self.ch = ord('s')

        return EmptyResponse()

if __name__ == '__main__':

    try:
        keyboard_control()
    except rospy.ROSInterruptException:
        pass

    
