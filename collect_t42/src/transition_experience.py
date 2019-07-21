
import numpy as np
import pickle
import os.path
import matplotlib.pyplot as plt
from scipy.io import savemat
import scipy.signal

version = '0'
Obj = 'obj'

class transition_experience():
    path = '/home/pracsys/catkin_ws/src/t42_control/hand_control/data/'

    def __init__(self, Load=True, discrete = True, postfix='', Object = Obj, with_fingers = False):

        if discrete:
            self.mode = 'd' # Discrete actions
        else:
            self.mode = 'c' # Continuous actions

        self.Object = Object
        self.with_fingers = with_fingers
        
        self.file_name = self.path + 'raw_' + self.Object + '_' + self.mode + '_v' + version + self.postfix + '.obj'

        if Load:
            self.load()
        else:
            self.clear()
        
    def add(self, time, state, action, next_state, done):
        self.memory += [(time, state, action, next_state, done)]
        
    def clear(self):
        self.memory = []

    def load(self):
        if os.path.isfile(self.file_name):
            print('Loading data from ' + self.file_name)
            with open(self.file_name, 'rb') as filehandler:
                self.memory = pickle.load(filehandler)
            print('Loaded transition data of size %d.'%self.getSize())
        else:
            self.clear()

    def getComponents(self):

        states = np.array([item[0] for item in self.memory])
        actions = np.array([item[1] for item in self.memory])
        next_states = np.array([item[2] for item in self.memory])

        return states, actions, next_states

    def save(self):
        print('Saving data...')
        file_pi = open(self.file_name, 'wb')
        pickle.dump(self.memory, file_pi)
        print('Saved transition data of size %d.'%self.getSize())
        file_pi.close()

    def getSize(self):
        return len(self.memory)

    def plot_data(self):

        T = np.array([item[0] for item in self.memory])
        states = np.array([item[1] for item in self.memory])
        states[:,:2] *= 1000.
        states[:,3:-2] *= 1000.
        done = np.array([item[4] for item in self.memory])
        failed_states = states[done]

        plt.figure(1)
        ax1 = plt.subplot(221)
        ax1.plot(states[:,3],states[:,4],'.b')
        ax1.plot(states[:,5],states[:,6],'.b')
        ax1.plot(states[:,7],states[:,8],'.b')
        ax1.plot(states[:,9],states[:,10],'.b')
        ax1.plot(states[:,0],states[:,1],'.y')
        ax1.plot(failed_states[:,0],failed_states[:,1],'.r')
        ax1.set(title='Object position')
        ax1.axis('equal')
        plt.ylim((-50, 140))
        plt.xlim((-60, 130))
        
        ax2 = plt.subplot(222)
        ax2.plot(states[:,-2],states[:,-1],'.k')
        ax2.plot(failed_states[:,-2],failed_states[:,-1],'.r')
        ax2.set(title='Actuator loads')
        ax2.axis('equal')
        plt.xlim((-10, 300))
        plt.ylim((-300, 10))

        ax3 = plt.subplot(223)
        ax3.plot(np.rad2deg(states[:,2]),'-k')
        ax3.set(title='Object angle')
        plt.ylim((-180., 180.))

        plt.show()



