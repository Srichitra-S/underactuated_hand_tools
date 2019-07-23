#!/usr/bin/env python

'''
Author: Avishai Sintov
        https://github.com/avishais
'''

'''
Node that subsrives to the sequence of actions and commands the actuators to move.
'''

import rospy
from std_msgs.msg import String, Float32MultiArray, Bool
from std_srvs.srv import Empty, EmptyResponse, SetBool
from rollout_t42.srv import TargetAngles
import numpy as np
import matplotlib.pyplot as plt
import pickle


class rollout():

    drop = True
    running = False
    action = np.array([0.,0.])
    suc = True
    drop_counter = 0

    def __init__(self):
        rospy.init_node('rollout_actor_t42', anonymous=True)

        self.move_srv = rospy.ServiceProxy('/MoveGripper', TargetAngles)
        rospy.Subscriber('/rollout/action', Float32MultiArray, self.callbackAction)
        fail_pub = rospy.Publisher('/rollout/fail', Bool, queue_size = 10)
        self.running_pub = rospy.Publisher('/rollout_actor/runnning', Bool, queue_size = 10)
        rospy.Service('/rollout/run_trigger', SetBool, self.callbackTrigger)

        # Marker detection - Needs to be implemented by the user
        rospy.Subscriber('/cylinder_drop', Bool, self.callbackObjectDrop)

        print('[rollout_actor] Ready to rollout...')
        self.running_pub.publish(False)

        self.rate = rospy.Rate(2.5) # 2.5hz limit due to communication band width
        while not rospy.is_shutdown():

            if self.running:
                self.suc = self.move_srv(self.action).success

                fail_pub.publish(not self.suc or self.drop)
               
                if not self.suc:
                    print("[rollout_actor] Load Fail")
                    self.running = False
                    self.running_pub.publish(False)
                elif self.drop:
                    print("[rollout_actor] Drop Fail")
                    self.running = False
                    self.running_pub.publish(False)

            self.rate.sleep()

    def callbackAction(self, msg):
        self.action = np.array(msg.data)

    def callbackObjectDrop(self, msg):
        if (msg.data):
            self.drop_counter += 1
        else:
            self.drop_counter = 0
        self.drop = (self.drop_counter >= 7)

    def callbackTrigger(self, msg):
        self.running = msg.data
        if self.running:
            print("[rollout_actor] Started ...")
            self.suc = True
            self.running_pub.publish(True)

        return {'success': True, 'message': ''}


if __name__ == '__main__':
    try:
        rollout()
    except rospy.ROSInterruptException:
        pass