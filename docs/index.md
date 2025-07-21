---
hide:
  - navigation
  - toc
---

<img src="media/crisp_logo.png" alt="CRISP Controllers Logo"/>

# CRISP - **C**a**R**tesian **I**mpedance and Operational **SP**ace Control for Learning-based Policies
*Authors: Daniel San Jose Pro, Oliver Hausdoerfer, Ralf Roemer, Martin Schuck and Angela Schoellig.*

> Collection of C++ controllers for torque-based control for manipulators compatible with `ros2_control`, including Operational Space Control and Cartesian Impedance Control. 

Check the [controllers (crisp_controllers) :simple-github:](https://github.com/utiasDSL/crisp_controllers) , robot [demos (crisp_controllers_demos) :simple-github:](https://github.com/utiasDSL/crisp_controllers_demos), a simple [python interface (crisp_py) :simple-github:](https://github.com/utiasDSL/crisp_py), and a [gym wrapper (crisp_gym) :simple-github:](https://github.com/utiasDSL/crisp_gym) for real-world experiments.

| <video src="media/pap_demo.mp4" controls="true" loop="true" autoplay="true" width="800"/> | <video src="media/policy.mp4" controls="true" loop="true" autoplay="true" width="800"/> |
|:--:|:--:|
| Robot teleoperated using a Follower-Leader system in [crisp_gym :simple-github:](https://github.com/utiasDSL/crisp_gym) | Diffusion Policy trained and deployed from the same demonstrations. | 


| ![Franka](media/franka.gif) | ![kinova](media/kinova.gif) | ![iiwa](media/iiwa.gif) |
|:--:|:--:|:--:|
| *Robot following a moving target, while base joint follows a sine curve.* | *Simulated kinova robot with continous joints and nullspace control* | *Another simulated robot example...* |

| ![franka_eight_reduced](media/franka_eight_reduced.gif)![franka_ns_reduced](media/franka_ns_reduced.gif) | ![vicon](media/franka_teleop.gif)|
|:--:|:--:|
| *Real robot following a target and being disturbed (contact) + null space control demonstration*  | *Demonstration using a cartesian controller teleoperated using Vicon tracking system (Speed x4)*| 


## Why?

Learning-based controllers, such as diffusion policies, deep reinforcement learning, and vision-action-models in general, typically output low-frequency or sporadic target poses, necessitating a low-level controller to track these references smoothly, especially in contact-rich environments.
While `ROS2` frameworks like `MoveIt` offer comprehensive motion planning capabilities, they are often unnecessarily complex for tasks requiring simple, real-time pose or joint servoing.

We present a set of lightweight, torque-based Cartesian and joint-space controllers implemented in C++ for `ros2_control`, compatible with any robot exposing an effort interface—a common standard among modern manipulators.
Our controllers incorporate friction compensation, joint limit avoidance, and error clipping, and have been validated on the Franka Robotics FR3 manipulator.

Designed for fast integration and real-time control, our implementation lowers the barrier to deploying learning-based algorithms on `ROS2`-compatible platforms.

## Features

- 🤖 Operational Space Controller as well as Cartesian Impedance Controller for torque-based control.  
- 🚫 No MoveIt or complicated path-planning, just a simple C++ `ros2_controller`. Ready to use.  
- ⚙️ Dynamically and highly parametrizable: powered by the `generate_parameter_library` you can modify stiffness and more during operation.  
- 🐍 Python interface to move your ROS2 robot around without having to think about topics, spinning, and more ROS2 concepts but without loosing the powerful ROS2 API. Check [crisp_py](https://github.com/utiasDSL/crisp_py) for more information and examples.
- 🔁 Gym environment with utilities to record trajectories in LeRobotFormat and deploy trained policies. Check [crisp_gym](https://github.com/utiasDSL/crisp_gym).
- ❓ Demos showcasing how to use the controller with FR3 of Franka Emika in single and bimanual setup. Check the [crisp_controller_demos](https://github.com/utiasDSL/crisp_controllers_demos).

## Citing

```bibtex
@inproceeding{
   TODO
}
```
