#!/usr/bin/env python

'''
Author: Avishai Sintov
        https://github.com/avishais
'''

'''
Commands the motion of the hand in auto or manual mode.
'''

import rospy
import numpy as np
import time
import random
from std_msgs.msg import String, Float32MultiArray, Bool
from std_srvs.srv import Empty, EmptyResponse
from hand_control.srv import observation, IsDropped, TargetAngles, RegraspObject, close
from transition_experience import *
# from common_msgs_gl.srv import SendBool, SendDoubleArray
import glob
from bowen_pose_estimate.srv import recordHandPose

class collect_data():

    gripper_closed = False
    trigger = True # Enable collection
    discrete_actions = False # Discrete or continuous actions
    arm_status = ' '
    global_trigger = True

    num_episodes = 0
    episode_length = 1000000 # !!!
    desired_action = np.array([0.,0.])
    drop = True

    A = np.array([[1.0,1.0],[-1.,-1.],[-1.,1.],[1.,-1.],[1.5,0.],[-1.5,0.],[0.,-1.5],[0.,1.5]])

    def __init__(self):
        rospy.init_node('collect_data', anonymous=True)

        rospy.Subscriber('/gripper/gripper_status', String, self.callbackGripperStatus)
        pub_gripper_action = rospy.Publisher('/collect/action', Float32MultiArray, queue_size=10)
        rospy.Service('/collect/trigger_episode', Empty, self.callbackManualTrigger)
        rospy.Service('/collect/save', Empty, self.callbackSave)
        self.recorderSave_srv = rospy.ServiceProxy('/actor/save', Empty)
        move_srv = rospy.ServiceProxy('/MoveGripper', TargetAngles)
        recorder_srv = rospy.ServiceProxy('/actor/trigger', Empty)

        # Reset object by the arm - These need to be implemented based on the object resetting setup.
        arm_reset_srv = rospy.ServiceProxy('/RegraspObject', RegraspObject) 
        rospy.Subscriber('/ObjectIsReset', String, self.callbackTrigger)

        # Marker detection - Needs to be implemented by the user
        rospy.Subscriber('/cylinder_drop', Bool, self.callbackObjectDrop)

        if rospy.has_param('~object_name'):
            self.discrete_actions = True if rospy.get_param('~actions_mode') == 'discrete' else False
            self.collect_mode = rospy.get_param('~collect_mode') # 'manual' or 'auto'

        if self.collect_mode == 'manual':
            rospy.Subscriber('/keyboard/desired_action', Float32MultiArray, self.callbackDesiredAction)
            ResetKeyboard_srv = rospy.ServiceProxy('/ResetKeyboard', Empty)

        close_srv = rospy.ServiceProxy('/CloseGripper', close)
        open_srv = rospy.ServiceProxy('/OpenGripper', Empty) 

        msg = Float32MultiArray()

        # msgd = record_srv()
        open_srv()
        time.sleep(2.)

        print('[collect_data] Ready to collect...')

        rate = rospy.Rate(2.5) # 15hz
        count_fail = 0
        self.first = False
        while not rospy.is_shutdown():

            if self.global_trigger:

                if self.collect_mode != 'manual':
                    if np.random.uniform() > 0.5:
                        self.collect_mode = 'shoot'
                        self.first = True
                    else:
                        self.collect_mode = 'auto'

                if not self.trigger and self.arm_status == 'waiting': # Start only when arm finished regrasping

                    if self.collect_mode == 'manual': 
                        ResetKeyboard_srv()
                    arm_reset_srv() 
                    print('[collect_data] Waiting for arm to grasp object...')
                    time.sleep(1.0)
                
                if self.arm_status != 'moving' and self.trigger:

                    print('[collect_data] Verifying grasp...')
                    if self.drop:
                        self.trigger = False
                        print('[collect_data] Grasp failed. Restarting')
                        open_srv()
                        count_fail += 1
                        if count_fail == 60: # Stop collecting if failed to regrasp after 60 trials
                            self.global_trigger = False
                        continue

                    count_fail = 0

                    print('[collect_data] Starting episode %d...' % self.num_episodes)

                    self.num_episodes += 1
                    Done = False

                    if self.collect_mode == 'shoot': # Go long distance right or left
                        if np.random.uniform() > 0.5:
                            Af = np.tile(np.array([-1.,1.]), (np.random.randint(100,500), 1))
                        else:
                            Af = np.tile(np.array([1.,-1.]), (np.random.randint(100,500), 1))
                        print('[collect_data] Rolling out shooting with %d steps.'%Af.shape[0])
                    
                    # Start episode
                    recorder_srv() # Start recording
                    n = 0
                    action = np.array([0.,0.])
                    T = rospy.get_time()
                    for ep_step in range(self.episode_length):
                        if self.collect_mode == 'shoot' and Af.shape[0] == ep_step and self.first: # Finished planned path and now applying random actions
                            self.first = False
                            n = 0
                            print('[collect_data] Running random actions...')
                        
                        if n == 0:
                            if self.collect_mode == 'auto':
                                action, n = self.choose_action()
                            elif self.collect_mode == 'manual':
                                action = self.desired_action
                                n = 1
                            else: # 'shoot'
                                if self.first:
                                    n = 1
                                    action = Af[ep_step, :] 
                                else:
                                    action, n = self.choose_action()                           
                        print action, ep_step
                        
                        msg.data = action
                        pub_gripper_action.publish(msg)
                        suc = move_srv(action).success
                        n -= 1

                        if suc:
                            fail = False
                            c = 0
                            while self.drop: # Verify drop
                                if c == 3:
                                    fail = True
                                    break
                                c += 1
                                rate.sleep()
                        else:
                            # End episode if overload or angle limits reached
                            rospy.logerr('[collect_data] Failed to move gripper. Episode declared failed.')
                            fail = True
                            recorder_srv()

                        if not suc or fail:
                            Done = True
                            break
                        
                        rate.sleep()
                
                    self.trigger = False
                    print('[collect_data] Finished running episode.')
                    print('[collect_data] Waiting for next episode initialization...')

                    if self.num_episodes > 0 and not (self.num_episodes % 5):
                        open_srv()
                        self.recorderSave_srv()
                        if (self.num_episodes % 50 == 0):
                            print('[collect_data] Cooling down.')
                            rospy.sleep(120)


    def callbackGripperStatus(self, msg):
        self.gripper_closed = msg.data == "closed"

    def callbackTrigger(self, msg):
        self.arm_status = msg.data
        if not self.trigger and self.arm_status == 'finished':
            self.trigger = True

    def callbackManualTrigger(self, msg):
        self.global_trigger = not self.global_trigger

    def callbackObjectDrop(self, msg):
        self.drop = msg.data

    def choose_action(self):
        if self.discrete_actions:
            a = self.A[np.random.randint(self.A.shape[0])]
        else:
            a = np.random.uniform(-1.,1.,2)
            
        if self.collect_mode == 'shoot':
            if self.first:
                n = np.random.randint(200)
            else:
                n = np.random.randint(15, 80)
                a = np.random.uniform(-1.,1.,2)
                print "Running " + str(n) + " times action " + str(a) + " ..."
        else:
            n = np.random.randint(60)    

        return a, n

    def callbackDesiredAction(self, msg):
        self.desired_action = msg.data

    def callbackSave(self, msg):
        self.recorderSave_srv()

if __name__ == '__main__':
    
    try:
        collect_data()
    except rospy.ROSInterruptException:
        pass
