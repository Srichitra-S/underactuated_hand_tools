#!/usr/bin/env python

'''
Author: Avishai Sintov
        https://github.com/avishais
'''

'''
Calls the data collection service.
'''

import rospy
from std_srvs.srv import Empty, EmptyResponse
import numpy as np
import matplotlib.pyplot as plt

save_srv = rospy.ServiceProxy('/collect/save_data', Empty)
rand_epi_srv = rospy.ServiceProxy('/collect/random_episode', Empty)


for i in range(1,100000):

        print "Running random episode..."
        rand_epi_srv()
        
    if not (i % 10):
        save_srv()

