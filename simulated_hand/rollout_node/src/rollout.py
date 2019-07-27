#!/usr/bin/env python

'''
Author: Avishai Sintov
        https://github.com/avishais
'''

'''
Provides service to rollout a sequence of actions. 
'''

import rospy
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Bool, String, Float32MultiArray
from rollout_node.srv import rolloutReq, observation, IsDropped, TargetAngles
import numpy as np
import matplotlib.pyplot as plt
import pickle
from rollout_node.srv import gets

class rollout():

    states = []
    actions = []
    plot_num = 0
    drop = True

    def __init__(self):
        rospy.init_node('rollout_node', anonymous=True)

        rospy.Service('/rollout/rollout', rolloutReq, self.CallbackRollout)
        self.action_pub = rospy.Publisher('/rollout/gripper_action', Float32MultiArray, queue_size = 10)

        self.obs_srv = rospy.ServiceProxy('/hand_control/observation', observation)
        self.drop_srv = rospy.ServiceProxy('/hand_control/IsObjDropped', IsDropped)
        self.move_srv = rospy.ServiceProxy('/hand_control/MoveGripper', TargetAngles)
        self.reset_srv = rospy.ServiceProxy('/hand_control/ResetGripper', Empty)
        rospy.Subscriber('/hand_control/cylinder_drop', Bool, self.callbackDrop)
        rospy.Subscriber('/hand_control/gripper_status', String, self.callbackGripperStatus)

        self.trigger_srv = rospy.ServiceProxy('/rollout_recorder/trigger', Empty)
        self.gets_srv = rospy.ServiceProxy('/rollout_recorder/get_states', gets)

        self.action_dim = 2
        self.stepSize = 1

        print("[rollout] Ready to rollout...")

        self.rate = rospy.Rate(2) 
        rospy.spin()

    def run_rollout(self, A):

        # Reset gripper
        while 1:
            self.reset_srv()
            while not self.gripper_closed:
                self.rate.sleep()
            break
        
        print("[rollout] Rolling-out...")

        msg = Float32MultiArray()

        # Start episode
        success = True
        self.trigger_srv()
        n = 0
        i = 0
        while 1:

            if n == 0:
                action = A[i,:]
                i += 1
                n = self.stepSize
            
            msg.data = action
            self.action_pub.publish(msg)
            suc = self.move_srv(action).success
            n -= 1
            
            if suc:
                fail = self.drop # Check if dropped - end of episode
            else:
                # End episode if overload or angle limits reached
                rospy.logerr('[rollout] Failed to move gripper. Episode declared failed.')
                fail = True

            if not suc or fail:
                print("[rollout] Fail.")
                success = False
                break
            
            if i == A.shape[0] and n == 0:
                print("[rollout] Complete.")
                success = True
                break

            self.rate.sleep()

        print("[rollout] Rollout done.")

        SA = self.gets_srv()
        self.states = SA.states
        self.actions = SA.actions 
        
        return success

    def callbackGripperStatus(self, msg):
        self.gripper_closed = msg.data == "closed"

    def callbackDrop(self, msg):
        self.drop = msg.data

    def CallbackRollout(self, req):
        
        actions_nom = np.array(req.actions).reshape(-1, self.action_dim)
        success = True
        success = self.run_rollout(actions_nom)

        return {'states': self.states, 'actions_res': self.actions, 'success' : success}


if __name__ == '__main__':
    try:
        rollout()
    except rospy.ROSInterruptException:
        pass