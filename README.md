:construction_worker: :construction: **_This page is under construction_** :construction: :construction_worker:


# Benchmarking for underactuated adaptive hands

This page provides instructions and source code for benchmarking underactuated adaptive hands. This is complementary material to the paper

> ***Modeling Benchmarks for Data-based Within-Hand Manipulation of
Underactuated Adaptive Hands***

submitted to the *IEEE Robotics and Automation Letters* special issue on *Benchmarking for Manipulations*.

**The code is based on ROS and tested on Kinetic.**

---
## RUM dataset

[Data](https://robotics.cs.rutgers.edu/rum-dataset/)

---


## Source code

### Model T42

1. To build a hand, follow instructions in the Yale OpenHand [project] (https://www.eng.yale.edu/grablab/openhand/).

2. Use Model T42 real hand ROS packages in `./ModelT42/`.

3. Load hand control node:
   - Setup hand parameters in `control_blue.yaml` and `model_t42_blue.yaml` in [link](https://github.com/avishais/underactuated_hand_benchmarking/tree/master/ModelT42/hand_control/param). 
   - Run:
        ```
        roslaunch hand_control run.launch
        ```

4. To collect data
   - Set the [settings yaml file](https://github.com/avishais/underactuated_hand_benchmarking/tree/master/ModelT42/collect_t42/param/settings.yaml) as desired. 
   - Run:

      ```
      roslaunch collect_t42 collect.launch
      ```

5. To rollout a sequence of actions, launch the required service
   ```
   roslaunch rollout_t42 rollout.launch
   ```
   Example usage of the rollout service is included in the package.

### Model O

1. Use Model O real hand ROS packages in `./ModelO/`.


### Physics-engine simulation

The Gazebo simulation package contains the Model-O (3-fingers), Model-T42 (2-fingers) and reflex (3-fingers) hands. However, currently the data collection and rollout package only supports the 2-fingers Model-T42 hand. Operation is similar 

1. Use simulated hand ROS packages in `./simulated_hand/`. 

2. Clone the simulation [repo](https://github.com/avishais/gazebo_adaptive_hand_simulator.git) and follow instructions to launch it.

3. To collect data
   - Set the [settings yaml file](https://github.com/avishais/underactuated_hand_benchmarking/tree/master/simulated_hand/collect_data/param/settings.yaml) as desired. 
   - Run:

      ```
      roslaunch collect_data collect.launch
      ```
   - Run:
      ```
      rosrun collect_data run.py
      ```

4. To rollout a sequence of actions, launch the required service
   ```
   roslaunch rollout_node rollout.launch
   ```
   Example usage of the rollout service is included in the package.







---
