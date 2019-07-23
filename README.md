:construction_worker: :construction:**_This page is under construction_**:construction: :construction_worker:


# Benchmarking for underactuated adaptive hands

This page provides instructions and source code for benchmarking underactuated adaptive hands. This is complementary material to the paper

> ***Modeling Benchmarks for Data-based Within-Hand Manipulation of
Underactuated Adaptive Hands***

submitted to the *IEEE Robotics and Automation Letters* special issue on *Benchmarking for Manipulations*.

**The code is based on ROS and tested in Kinetic.**

---
## RUM dataset

---


## Real-hand code

### Model T42

1. Use Model T42 real hand ROS packages in `./ModelT42/`.

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

1. Use Model O real hand ROS packages in `./ModelO/`.


## Physics-engine simulation

The Gazebo simulation package contains the Model-O (3-fingers), Model-T42 (2-fingers) and reflex (3-fingers) hands. However, currently the data collection and rollout package only supports the 2-fingers Model-T42 hand.

1. Use simulated hand ROS packages in `./simulated_hand/`. 

2. Clone the simulation [repo](https://github.com/avishais/gazebo_adaptive_hand_simulator.git) and follow instructions to launch it.







---
