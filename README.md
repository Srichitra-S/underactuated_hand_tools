:construction_worker: :construction: **_This page is under construction_** :construction: :construction_worker:


# Benchmarking for underactuated adaptive hands

This page provides instructions and source code for benchmarking underactuated adaptive hands. This is complementary material to the paper

> ***Benchmarking Data-based Within-Hand Manipulation with Underactuated Adaptive Hands***

submitted to the *IEEE Robotics and Automation Letters* special issue on *Benchmarking for Manipulations*.

**The code is based on ROS and tested on Kinetic.**

To build the hands, follow instructions in the [Yale OpenHand project](https://www.eng.yale.edu/grablab/openhand/).

We provide a benchmarking platform in three paths. The user can either use the *Rutgers Underactuated-hand Manipulation* (RUM) dataset, independently collect data on a real system, or collect in a simulated environment. We provide the RUM dataset and source code for the user to take on any of the paths.  

---
## RUM dataset

The Rutgers Underactuated-hand Manipulation (RUM) dataset is a wide set of real data collected from several underactuated hands (the Model-T42 and Model-O) across various types of objects. The data comprises of approximately 300,000 transition points for each object. Furthermore, each objects comes with 10 long validation paths which have substantial coverage of the hand's workspace and also employs frequent changes of action. 

**The data can be accessed [here](https://robotics.cs.rutgers.edu/rum-dataset/).**

Example for accessing a transition data files:
```python
import numpy as np
import pickle

with open('raw_t42_cyl35_d.obj', 'rb') as f: 
    episodes = pickle.load(f)

for e in episodes:
    i = e['episode_number'] # Episode number
    T = e['time_stamps'] # N time stamps for points in the episode <np.array>
    S = e['states'] # Numpy array (N x 14) of states
    A = e['actions'] # Numpy array (N x 2) of actions
    S_next = e['next_states'] # Numpy array (N x 14) of next states
```

A state is formed in the following format:
```
[<object position {x,y} (2)>, <object orientation (1)>, <finger positions {x,y,x,y,...} (8), gripper (actuators) load (2)>]
```

Example for accessing a failure data files:
```python
with open('classifier_data_t42_cyl35_d.obj', 'rb') as f: 
    D = pickle.load(f)

SA_train = D['train_data'] # Training state-action pairs - numpy array [states, actions] (N x 15)
label_train = D['train_labels'] # Training labels - True for failed and False for normal
SA_test = D['test_data']  # Test state-action pairs - numpy array [states, actions] (N x 15)
label_test = D['test_labels'] # Testing labels - True for failed and False for normal
A = D['test_action_seq'] # List of action sequences to replicate the failure data
```
---

## Source code

### Model-T42

1. Use Model-T42 real hand ROS packages in `./ModelT42/`.

2. Load hand control node:
   - Setup hand parameters in `control_blue.yaml` and `model_t42_blue.yaml` in [link](https://github.com/avishais/underactuated_hand_benchmarking/tree/master/ModelT42/hand_control/param). 
   - Run:
        ```
        roslaunch hand_control run.launch
        ```

3. To collect data
   - Set the [settings yaml file](https://github.com/avishais/underactuated_hand_benchmarking/tree/master/ModelT42/collect_t42/param/settings.yaml) as desired. 
   - Run:

      ```
      roslaunch collect_t42 collect.launch
      ```

4. To rollout a sequence of actions, launch the required service
   ```
   roslaunch rollout_t42 rollout.launch
   ```
   Example usage of the rollout service is included in the package.


### Model-O

1. Use Model O real hand ROS packages in `./ModelO/`.

2. Load hand control node:
   - Setup hand parameters in `control_o.yaml` and `model_o.yaml` in [link](https://github.com/avishais/underactuated_hand_benchmarking/tree/master/ModelO/hand_control/param). 
   - Run:
        ```
        roslaunch hand_control run.launch
        ```

3. Collection for the Model-O is currently done manually through the keyboard. 

    - Run in another terminal:
        ```
        rosrun hand_control keyboard_control.py
        ```
    - Use keys as described in the python code to control hand and manipulate the object. Press 'u' or 'j' to start or stop recording an episode, respectively.

---

## Physics-engine simulation

The Gazebo simulation package contains the Model-O (3-fingers), Model-T42 (2-fingers) and reflex (3-fingers) hands. However, currently the data collection and rollout package only supports the 2-fingers Model-T42 hand. Operation is similar to the real hand package.

1. Use simulated hand ROS packages in `./simulated_hand/`. 

2. Clone the simulation [repo](https://github.com/avishais/gazebo_adaptive_hand_simulator.git) and follow instructions to launch it.

3. To collect data
   - Set the [settings yaml file](https://github.com/avishais/underactuated_hand_benchmarking/tree/master/simulated_hand/collect_data/param/settings.yaml) as desired. 
   - Run:

      ```
      roslaunch collect_data collect.launch
      ```
   - In a different terinal, run:
      ```
      rosrun collect_data run.py
      ```

4. To rollout a sequence of actions, launch the service with
   ```
   roslaunch rollout_node rollout.launch
   ```
   Example usage of the rollout service is included in the package.

---
