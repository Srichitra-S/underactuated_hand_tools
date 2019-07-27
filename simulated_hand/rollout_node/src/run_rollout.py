#!/usr/bin/env python

'''
Author: Avishai Sintov
        https://github.com/avishais
'''

'''
Example code to rollout and plot a set of action files (containing n x 2 matrices) stored in text files.
'''

import rospy
from std_srvs.srv import Empty, EmptyResponse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Polygon
import pickle
from rollout_node.srv import rolloutReq
import time
import glob
from scipy.io import savemat, loadmat

rollout_srv = rospy.ServiceProxy('/rollout/rollout', rolloutReq)

rospy.init_node('run_rollout_set', anonymous=True)
state_dim = 14

path = './'

rollout = 1

############################# Rollout ################################
if rollout:

    files = glob.glob(path + "*.txt")

    for F in range(files):

        action_file = F
        matfile = action_file[:-3]
        if action_file.find('traj') > 0 or action_file.find('roll') > 0:
            continue

        print('Rolling-out file: ' + action_file + '.')

        A = np.loadtxt(action_file, delimiter=',', dtype=float)[:,:2]

        Af = A.reshape((-1,))
        Pro = []
        for j in range(5):
            print("Rollout number " + str(j) + ".")
            
            Sro = np.array(rollout_srv(Af).states).reshape(-1,state_dim)

            Pro.append(Sro)

            with open(matfile + 'pkl', 'w') as f: 
                pickle.dump(Pro, f)

############################# Filter ################################

def medfilter(x, W):
    w = int(W/2)
    x_new = np.copy(x)
    for i in range(0, x.shape[0]):
        if i < w:
            x_new[i] = np.mean(x[:i+w])
        elif i > x.shape[0]-w:
            x_new[i] = np.mean(x[i-w:])
        else:
            x_new[i] = np.mean(x[i-w:i+w])
    return x_new

############################# Plot ################################

if not rollout:
    files = glob.glob(path + "*.pkl")
    K = []
    for k in range(len(files)):
        pklfile = files[k]
        for j in range(len(pklfile)-1, 0, -1):
            if pklfile[j] == '/':
                break
        file_name = pklfile[j+1:-4]
        if file_name == 'rollout_output':
            continue
        print('Plotting file number ' + str(k) + ': ' + file_name)

        Straj = np.loadtxt(pklfile[:-8] + 'traj.txt', delimiter=',', dtype=float)[:,:2]
        
        with open(pklfile) as f:  
            Pro = pickle.load(f)

        fig = plt.figure(k)

        for i in range(len(Pro)):
            K.append(Pro[i][0,:]*1000)
            for i in range(4):
                S[:,i] = medfilter(S[:,i], 20)
            plt.plot(S[:,0], S[:,1], '.-b', label='rollout')
            plt.plot(S[0,0], S[0,1], 'oc', label='rollout')

        np.savetxt(pklfile[:-8] + 'roll.txt', S, delimiter=',')
        
        plt.plot(Straj[:,0], Straj[:,1], '.-r', label='planned')
        plt.plot(Straj[0,0], Straj[0,1], 'o-r', label='planned')

    plt.show()

