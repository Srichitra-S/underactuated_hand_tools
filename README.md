# Benchmarking for underactuated adaptive hands

This page provides instructions and source code for benchmarking underactuated adaptive hands. This is complementary material to the paper

> ***Modeling Benchmarks for Data-based Within-Hand Manipulation of
Underactuated Adaptive Hands***

submitted to the *IEEE Robotics and Automation Letters* special issue on *Benchmarking for Manipulations*.

---
## RUM dataset

---

All code is based on ROS and tested in Kinetic.

## Real-hand code

### Model T42

1. Switch to the real hand code

   ```
   git checkout master
   ``` 

2. Load hand control node
   
   ```
   roslaunch hand_control run.launch
   ```

3. To collect data
   - Set the [settings yaml file](https://github.com/avishais/underactuated_hand_benchmarking/tree/master/collect_t42/param) as desired. 
   - Run:

      ```
      roslaunch collect_t42 collect.launch
      ```

4. To rollout a sequence of actions, launch the required service
   ```
   roslaunch rollout_t42 rollout.launch
   ```

### Model O

## Physics-engine simulation

1. Switch to the simulation branch: 

   ```
   git checkout simulation
   ``` 

2. Clone the simulation [repo](https://github.com/avishais/gazebo_adaptive_hand_simulator.git) and follow instructions to launch it.







---
