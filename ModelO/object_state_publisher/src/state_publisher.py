#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Quaternion
from bowen_pose_estimate.msg import multifacePose
from std_msgs.msg import String,Float64MultiArray
import numpy as np 
from tf.transformations import euler_from_quaternion#, quaternion_from_euler
from std_srvs.srv import Empty, EmptyResponse


class state_listener():

    obj_pose = np.array([0.,0.,0.,0.,0.,0.])

    def __init__(self):
        rospy.init_node('object_state_listener', anonymous=True)

        rospy.Subscriber('/marker_pose', multifacePose, self.callbackmultifacePose)
        # rec_srv = rospy.ServiceProxy('/record_hand_pose', Empty)
        pose_pub = rospy.Publisher('/object_pose', Float64MultiArray, queue_size=15)

        # rospy.sleep(2.0)
        # rec_srv()

        print('[object_state_listener] Publishing object pose...')
        msg = Float64MultiArray()

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            msg.data = self.obj_pose
            pose_pub.publish(msg)
            rate.sleep()
        

    def callbackmultifacePose(self, msg):

        # Position
        id = msg.id.data
        if id == 6:
            x = msg.pose.position.x - (.64/2)
            y = msg.pose.position.y - (.30/2)
            z = msg.pose.position.z - (.60/2)
        elif id == 2:
            x = msg.pose.position.x - (.64/2)
            y = msg.pose.position.y - (.30/2)
            z = msg.pose.position.z - (.60/2)
        elif id == 4:
            x = msg.pose.position.x - (.60/2)
            y = msg.pose.position.y - (.30/2)
            z = msg.pose.position.z - (.64/2)
        elif id == 0:
            x = msg.pose.position.y# - (.60/2)
            y = msg.pose.position.x# - (.30/2)
            z = msg.pose.position.z# - (.64/2)
        else:
            print('[object_state_listener] Cannot identify marker...')

        # Orientation
        # orientation = msg.pose.orientation
        roll, pitch, yaw = 0,0,0#euler_from_quaternion(orientation)

        self.obj_pose = np.array([x*1e3, y*1e3, z*1e3, roll, pitch, yaw])

        print self.obj_pose

if __name__ == '__main__':
    
    try:
        state_listener()
    except rospy.ROSInterruptException:
        pass