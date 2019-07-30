#!/usr/bin/python 

'''
----------------------------
Author: Avishai Sintov
        Rutgers University
----------------------------
'''

'''
This node provides general services to control the Model O hand.
'''

import rospy
import numpy as np 
from std_msgs.msg import Float64MultiArray, Float32MultiArray, String, Bool, Float32
from std_srvs.srv import Empty, EmptyResponse
from openhand.srv import *
from hand_control.srv import TargetAngles, IsDropped, observation, close, ObjOrientation, MovePrim

import geometry_msgs.msg
import math
import time
import pickle

class hand_control():

    finger_initial_offset = np.array([0., 0., 0., 0.])
    finger_opening_position = np.array([0.0, 0.0, 0.0, 0.0])
    finger_closing_position = np.array([0., 0., 0.0, 0.0])
    finger_move_offset = np.array([0.01, 0.01, 0.01, 0.01])
    closed_load = np.array(20.)

    gripper_pos = np.array([0., 0., 0., 0.])
    gripper_load = np.array([0., 0., 0., 0.])
    gripper_temperature = np.array([0., 0., 0., 0.])
    base_pos = [0,0]
    base_theta = 0
    obj_pos = [0,0]
    R = []
    count = 1
    vel_ref = np.array([0.,0.])
    gripper_cur_pos = np.array([0.,0.])
    max_load = 280.0

    gripper_status = 'open'
    object_grasped = False
    drop_query = True

    move_servos_srv = 0.
    df = 0.0
    
    memory = []
    Action = np.array([0.,0.,0.,0.])
    record_trigger = False
    T = 0.0

    path = './'

    def __init__(self):
        rospy.init_node('hand_control', anonymous=True)
        
        if rospy.has_param('~finger_initial_offset'):
            self.finger_initial_offset = rospy.get_param('~finger_initial_offset')
            self.finger_opening_position = rospy.get_param('~finger_opening_position')
            self.finger_closing_position = rospy.get_param('~finger_closing_position')
            self.finger_move_offset = rospy.get_param('~finger_move_offset')
            self.closed_load = rospy.get_param('~finger_close_load')

        rospy.Subscriber('/gripper/pos', Float32MultiArray, self.callbackGripperPos)
        rospy.Subscriber('/gripper/load', Float32MultiArray, self.callbackGripperLoad)
        rospy.Subscriber('/gripper/temperature', Float32MultiArray, self.callbackGripperTemp)
        
        pub_gripper_status = rospy.Publisher('/gripper/gripper_status', String, queue_size=10)
        pub_drop = rospy.Publisher('/hand_control/drop', Bool, queue_size=10)
        pub_obj_pos = rospy.Publisher('/hand_control/obj_pos_mm', Float32MultiArray, queue_size=10)
        pub_obj_orientation = rospy.Publisher('/object_orientation', Float32MultiArray, queue_size=10)

        # Marker detection - Needs to be implemented by the user
        rospy.Subscriber('/cylinder_pose', geometry_msgs.msg.Pose, self.callbackMarkers)
        rospy.Subscriber('/cylinder_drop', Bool, self.callbackObjectDrop)

        rospy.Service('/OpenGripper', Empty, self.OpenGripper)
        rospy.Service('/CloseGripper', close, self.CloseGripper)
        rospy.Service('/Spread', TargetAngles, self.SpreadGripper)
        rospy.Service('/MoveGripper', TargetAngles, self.MoveGripper)
        rospy.Service('/IsObjDropped', IsDropped, self.CheckDroppedSrv)
        rospy.Service('/observation', observation, self.GetObservation)
        rospy.Service('/MoveKeys', MovePrim, self.MoveKeys)
        rospy.Service('/MovePrimitives', MovePrim, self.MovePrimitives)

        self.move_servos_srv = rospy.ServiceProxy('/MoveServos', MoveServos)
        self.moveReqservo_srv = rospy.ServiceProxy('/MoveReqServos', MoveReqServos)
        self.temperature_srv = rospy.ServiceProxy('/ReadTemperature', ReadTemperature)

        msg = Float32MultiArray()

        self.rate = rospy.Rate(10)
        c = True
        count = 0
        while not rospy.is_shutdown():
            pub_gripper_status.publish(self.gripper_status)

            msg.data = self.obj_pos
            pub_obj_pos.publish(msg)

            #publishing the angle of the object
            msg.data = self.angle
            pub_obj_orientation.publish(msg)

            if count > 1000:
                dr, verbose = self.CheckDropped()
                pub_drop.publish(dr)
            count += 1

            if self.record_trigger:
                self.record()

            if c and not np.all(self.gripper_load==0): # Wait till openhand services ready and set gripper open pose
                self.moveGripper(self.finger_opening_position)
                for i in range(50):
                    self.rate.sleep()
                self.gripper_cur_pos = np.copy(self.gripper_pos)
                self.gripper_cur_pos[0] = self.finger_opening_position[0]
                c = False

            self.rate.sleep()

    def record(self):
        time = rospy.get_time()-self.T
        self.memory += [(time, self.Action)]
    
    def callbackGripperPos(self, msg):
        self.gripper_pos = np.array(msg.data)

    def callbackGripperLoad(self, msg):
        self.gripper_load = np.array(msg.data)
    
    def callbackGripperTemp(self, msg):
        self.gripper_temperature = np.array(msg.data)

    def callbackMarkers(self, msg):
        try:
            self.obj_pos = np.array([msg.position.x, msg.position.y, msg.position.z, , msg.orientation.z, , msg.orientation.y, , msg.orientation.z]) # Position and Euler angles
        except:
            self.obj_pos = np.array([np.nan, np.nan, np.nan, np.nan, np.nan, np.nan])

    def callbackObjectDrop(self, msg):
        self.drop_query = msg.data

    def OpenGripper(self, msg):

        open_pos = self.finger_opening_position
        open_pos[0] = self.gripper_pos[0]

        self.moveGripper(self.finger_opening_position, open=True)

        self.gripper_status = 'open'
        for i in range(50):
            self.rate.sleep()
        self.gripper_cur_pos = np.copy(self.gripper_pos)
        print "Open ", self.gripper_pos, self.gripper_cur_pos

        return EmptyResponse()

    def CloseGripper(self, msg):
        # self.vel_ref = np.array([0.,0.,])
        
        if np.any(self.gripper_temperature > 52.):
            rospy.logerr('[hand_control] Actuators overheated, taking a break...')
            while 1:
                if np.all(self.gripper_temperature < 45.):
                    break
                self.rate.sleep()

        closed_load = self.closed_load#np.random.randint(70, self.closed_load+30) # !!!!!! Remember to change

        self.object_grasped = False
        Ex = [[0,3],[0,1,2]]
        # Close pinching fingers
        for i in range(100):
            # print('Angles: ' + str(self.gripper_pos) + ', load: ' + str(self.gripper_load), self.closed_load)

            if np.any(abs(self.gripper_load[Ex[1]]) > closed_load):
                c = 0
                for k in range(10):
                    if np.any(abs(self.gripper_load[Ex[1]]) > closed_load):
                        c += 1
                    self.rate.sleep()
                if c > 7:
                    self.gripper_status = 'closed'
                    # rospy.loginfo('[hand] Object grasped.')
                    break

            inc = np.array(self.finger_move_offset)/2. # Move only fingers that haven't reached the load
            inc[Ex[0]] = 0.0

            desired = self.gripper_pos + inc
            print self.gripper_pos, inc, desired
            if np.any(desired[1:] > 0.75) or np.any(self.gripper_load > 220):
                rospy.logerr('[hand] Desired angles out of bounds or pre-overload.')
                break
            # print self.gripper_pos, desired
            self.moveGripper(desired)
            # self.rate.sleep()

        # Close thumb
        if self.gripper_status == 'closed':
            self.gripper_status = 'open'
            for i in range(100):
                # print('Angles: ' + str(self.gripper_pos) + ', load: ' + str(self.gripper_load), self.closed_load)
                d = 1.
                if abs(self.gripper_load[Ex[0][1]]) > closed_load / d:
                    c = 0
                    for k in range(10):
                        if abs(self.gripper_load[Ex[0][1]]) > closed_load / d:
                            c += 1
                        self.rate.sleep()
                    if c > 9:
                        self.gripper_status = 'closed'
                        rospy.loginfo('[hand] Object grasped.')
                        break

                desired = self.gripper_pos[Ex[0][1]] + self.finger_move_offset[Ex[0][1]]/2
                if desired > 0.75 or np.any(self.gripper_load > 220):
                    rospy.logerr('[hand] Desired angle out of bounds or pre-overload.')
                    break
                self.moveReqservo_srv(np.array([desired]), np.array([Ex[0][1]]))
                self.rate.sleep()


        self.rate.sleep()

        ## Verify based on gripper motor angles
        print('[hand] Gripper actuator angles: ' + str(self.gripper_pos))
        self.object_grasped = self.gripper_status == 'closed'

        self.rate.sleep()
        self.gripper_cur_pos = self.gripper_pos
        self.drop_query = True

        return {'success': self.object_grasped}

    def SpreadGripper(self, msg):
        goal_pos = self.gripper_pos
        goal_pos[0] = np.array(msg.angles)[0]
        self.moveGripper(goal_pos, open=True)

        return True

    def MoveGripper(self, msg):
        # This function should accept a vector of normalized increments to the current angles: msg.angles = [dq1, dq2], where dq1 and dq2 can be equal to 0 (no move), 1,-1 (increase or decrease angles by finger_move_offset)
        f = 100.0

        inc = np.array(msg.angles)
        inc_angles = np.multiply(self.finger_move_offset, inc)
        # suc = True

        self.gripper_cur_pos += inc_angles*1.0/f
        suc = self.moveGripper(self.gripper_cur_pos)

        return {'success': suc}
    
    def moveGripper(self, angles, open=False):
        if not open:
            for angle in angles:
                if angle > 1.0 or angle < 0.00:
                    rospy.logerr('[hand] Move Failed. Desired angles out of bounds.')
                    return False
            for L in self.gripper_load:
                if abs(L) > self.max_load:
                    rospy.logerr('[hand] Move failed. Pre-overload.')
                    return False

        self.move_servos_srv.call(angles)

        return True

    def MovePrimitives(self, req):
        action = req.action
        
        # Stop
        if action == 's':
            inc_angles = np.multiply(self.finger_move_offset, np.array([0., 0., 0., 0.]))
        # Up
        if action == 'w': 
            inc_angles = np.multiply(self.finger_move_offset, np.array([0., -0.6, -0.6, 1.0]))
        # Down
        if action == 'x':
            inc_angles = np.multiply(self.finger_move_offset, np.array([0., 1., 1., -0.7]))
        # Tilt right
        if action == 'd':
            inc_angles = np.multiply(self.finger_move_offset, np.array([0., 1., -1., 0.]))
        # Tilt Left
        if action == 'a':
            inc_angles = np.multiply(self.finger_move_offset, np.array([0., -1., 1., 0.]))
        # Close all
        if action == 'v':
            inc_angles = np.multiply(self.finger_move_offset, np.array([0., 1., 1., 1.]))
        # Open all
        if action == 'r':
            inc_angles = np.multiply(self.finger_move_offset, np.array([0., -1., -1., -1.]))

        # Finger 1 - Open
        if action == '1':
            inc_angles = np.multiply(self.finger_move_offset, np.array([0., -1., 0., 0.]))
        # Finger 2 - Open
        if action == '2':
            inc_angles = np.multiply(self.finger_move_offset, np.array([0., 0., -1., 0.]))
        # Finger 3 - Open
        if action == '3':
            inc_angles = np.multiply(self.finger_move_offset, np.array([0., 0., 0., -1.]))
        # Finger 1 - Close
        if action == '4':
            inc_angles = np.multiply(self.finger_move_offset, np.array([0., 1., 0., 0.]))
        # Finger 2 - Close
        if action == '5':
            inc_angles = np.multiply(self.finger_move_offset, np.array([0., 0., 1., 0.]))
        # Finger 3 - Close
        if action == '6':
            inc_angles = np.multiply(self.finger_move_offset, np.array([0., 0., 0., 1.]))
        # Spread - Open
        if action == '7':
            inc_angles = np.multiply(self.finger_move_offset, np.array([-1., 0., 0., 0.]))*4.
        # Spread - Close
        if action == '9':
            inc_angles = np.multiply(self.finger_move_offset, np.array([1., 0., 0., 0.]))*4.

        # Pull upper fingers
        if action == 'e':
            inc_angles = np.multiply(self.finger_move_offset, np.array([0., 1., 1., 0.]))
        # Release upper fingers
        if action == 'c':
            inc_angles = np.multiply(self.finger_move_offset, np.array([0., -1., -1., 0.]))

        # Power close
        if action == 'y':
            inc_angles = np.multiply(self.finger_move_offset, np.array([0., 1., 1., 1.])*6)

        f = 15.0
        self.gripper_cur_pos += inc_angles*1.0/f
        self.gripper_cur_pos[ np.where( self.gripper_cur_pos < 0.0 )[0]] = 0.0
        self.gripper_cur_pos[ np.where( self.gripper_cur_pos > 1.0 )[0]] = 1.0
        suc = self.moveGripper(self.gripper_cur_pos)

        self.Action = np.copy(inc_angles)
        return suc


    def MoveKeys(self, req):
        action = req.action

        # Open and close
        if action == 'o':
            msg = close()
            self.CloseGripper(msg)
            return True
        if action == 'p':
            msg = Empty()
            self.OpenGripper(msg)
            return True

        # Start recording
        if action == 'u':
            self.record_trigger = True 
            self.memory = []
            self.T = rospy.get_time()
            return True
        # Stop recording
        if action == 'j':
            self.record_trigger = False 
            with open(self.path + 'modelO_actionSeq_d_roll_' + str(np.random.randint(100000)) + '.pkl', 'wb') as f: 
                pickle.dump(self.memory, f)
            return True
        
        if np.any(action == np.array(['n','m'])):
            if action == 'n':
                self.df += 1.0
                print "Increased velocity by 1"
            if action == 'm':
                self.df -= 1.0
                print "Decreased velocity by 1"
        
        return False       

    def CheckDropped(self):
        # Should spin (update topics) between moveGripper and this

        if self.drop_query:
            verbose = '[hand] Object dropped.'
            return True, verbose

        if self.gripper_pos[0] > 0.9 or self.gripper_pos[1] > 0.9 or self.gripper_pos[0] < 0.03 or self.gripper_pos[1] < 0.03:
            verbose = '[hand] Desired angles out of bounds.'
            return True, verbose

        # Check load
        if abs(self.gripper_load[0]) > self.max_load or abs(self.gripper_load[1]) > self.max_load:
            verbose = '[hand] Pre-overload.'
            return True, verbose
        
        return False, ''

    def CheckDroppedSrv(self, msg):

        dr, verbose = self.CheckDropped()
        
        if len(verbose) > 0:
            rospy.logerr(verbose)

        return {'dropped': dr}


    def GetObservation(self, msg):
        obs = np.concatenate((self.obj_pos, self.angle,self.marker0,self.marker1,self.marker2,self.marker3, self.gripper_load))
        return {'state': obs}



if __name__ == '__main__':
    
    try:
        hand_control()
    except rospy.ROSInterruptException:
        pass
