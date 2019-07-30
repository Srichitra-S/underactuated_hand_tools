#!/usr/bin/python 

'''
Author: Avishai Sintov
        https://github.com/avishais
'''

'''
This node provides general services to control the Model T42 hand.
'''


import rospy
import numpy as np 
from std_msgs.msg import Float64MultiArray, Float32MultiArray, String, Bool, Float32
from std_srvs.srv import Empty, EmptyResponse
from openhand.srv import MoveServos, ReadTemperature
from hand_control.srv import TargetAngles, IsDropped, observation, close

import geometry_msgs.msg
import math
import time

class hand_control():

    finger_initial_offset = np.array([0., 0.])
    finger_opening_position = np.array([0.2, 0.33])
    finger_closing_position = np.array([0., 0.])
    finger_move_offset = np.array([0.01, 0.01])
    closed_load = np.array(20.)

    gripper_pos = np.array([0., 0.])
    gripper_load = np.array([0., 0.])
    gripper_temperature = np.array([0., 0.])
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

    angle = np.array([0.])
    marker0 = np.array([0.,0.])
    marker1 = np.array([0.,0.])
    marker2 = np.array([0.,0.])
    marker3 = np.array([0.,0.])
    cornerPos = []
    
    move_servos_srv = 0.

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
        rospy.Subscriber('/cylinder_pose', geometry_msgs.msg.Pose, self.callbackMarkers) # Get object marker center position
        rospy.Subscriber('cylinder_drop', Bool, self.callbackObjectDrop) # True - object dropped from hand
        rospy.Subscriber('/finger_markers', geometry_msgs.msg.PoseArray, self.callAddFingerPos) # Get finger marker center positions
        rospy.Subscriber('/cylinder_corner',geometry_msgs.msg.Pose,self.getCorner) # Get corner of object marker to calculate orientation

        rospy.Service('/OpenGripper', Empty, self.OpenGripper)
        rospy.Service('/CloseGripper', close, self.CloseGripper)
        rospy.Service('/MoveGripper', TargetAngles, self.MoveGripper)
        rospy.Service('/IsObjDropped', IsDropped, self.CheckDroppedSrv)
        rospy.Service('/observation', observation, self.GetObservation)

        self.move_servos_srv = rospy.ServiceProxy('/MoveServos', MoveServos)
        self.temperature_srv = rospy.ServiceProxy('/ReadTemperature', ReadTemperature)

        msg = Float32MultiArray()

        self.rate = rospy.Rate(100)
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

            if c and not np.all(self.gripper_load==0): # Wait till openhand services ready and set gripper open pose
                self.moveGripper(self.finger_opening_position)
                c = False

            self.rate.sleep()
    
    def getCorner(self,msg):
        self.cornerPos = [msg.position.x, msg.position.y]
        arr1 = self.cornerPos
        arr2 = self.obj_pos
        self.angle[0] = np.arctan2((arr1[1]-arr2[1]),(arr1[0]-arr2[0]))
        self.angle = np.array(self.angle)

    def callbackGripperPos(self, msg):
        self.gripper_pos = np.array(msg.data)

    def callbackGripperLoad(self, msg):
        self.gripper_load = np.array(msg.data)
    
    def callbackGripperTemp(self, msg):
        self.gripper_temperature = np.array(msg.data)

    def callbackMarkers(self, msg):
        try:
            self.obj_pos = np.array([msg.position.x, msg.position.y])
            self.obj_height = msg.position.z
        except:
            self.obj_pos = np.array([np.nan, np.nan])
            self.obj_height = np.nan

    def callbackObjectDrop(self, msg):
        self.drop_query = msg.data

    def callAddFingerPos(self, msg):
        tempMarkers =  msg.poses
        
        self.marker0[0] = tempMarkers[0].position.x
        self.marker0[1] = tempMarkers[0].position.y

        self.marker1[0] = tempMarkers[1].position.x
        self.marker1[1] = tempMarkers[1].position.y

        self.marker2[0] = tempMarkers[2].position.x
        self.marker2[1] = tempMarkers[2].position.y 

        self.marker3[0] = tempMarkers[3].position.x
        self.marker3[1] = tempMarkers[3].position.y

    def OpenGripper(self, msg):
        self.moveGripper(self.finger_opening_position, open=True)
        self.gripper_status = 'open'
        return EmptyResponse()

    def CloseGripper(self, msg):
        if np.any(self.gripper_temperature > 52.):
            rospy.logerr('[hand_control] Actuators overheated, taking a break...')
            while 1:
                if np.all(self.gripper_temperature < 60.):
                    break
                self.rate.sleep()

        closed_load = self.closed_load

        self.object_grasped = False
        for i in range(100):
            if abs(self.gripper_load[0]) > closed_load or abs(self.gripper_load[1]) > closed_load:
                rospy.loginfo('[hand] Object grasped.')
                self.gripper_status = 'closed'
                break

            desired = self.gripper_pos + np.array([ a*4.0 for a in self.finger_move_offset])/18. 
            if desired[0] > 0.7 or desired[1] > 0.7:
                rospy.logerr('[hand] Desired angles out of bounds.')
                break
            self.moveGripper(desired)
            rospy.sleep(0.2)  

        self.rate.sleep()

        ## Verify based on gripper motor angles
        print('[hand] Gripper actuator angles: ' + str(self.gripper_pos))
        self.object_grasped = True

        self.rate.sleep()
        self.gripper_cur_pos = self.gripper_pos

        return {'success': self.object_grasped}

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
            if angles[0] > 0.9 or angles[1] > 0.9 or angles[0] < 0.02 or angles[1] < 0.02:
                rospy.logerr('[hand] Move Failed. Desired angles out of bounds.')
                return False

            if abs(self.gripper_load[0]) > self.max_load or abs(self.gripper_load[1]) > self.max_load:
                rospy.logerr('[hand] Move failed. Pre-overload.')
                return False

        self.move_servos_srv.call(angles)

        return True

    def CheckDropped(self):

        if self.drop_query:
            verbose = '[hand] Object dropped.'
            return True, verbose

        try:
            if self.gripper_pos[0] > 0.9 or self.gripper_pos[1] > 0.9 or self.gripper_pos[0] < 0.03 or self.gripper_pos[1] < 0.03:
                verbose = '[hand] Desired angles out of bounds.'
                return True, verbose
        except:
            print('[hand] Error with gripper_pos.')

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
