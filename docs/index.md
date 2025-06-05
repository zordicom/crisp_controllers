---
hide:
  - navigation
  - toc
---

<img src="media/crisp_logo.png" alt="CRISP Controllers Logo" width="160" align="right"/>
# CRISP - **C**a**R**tesian **I**mpedance and Operational **SP**ace control for robotic arm manipulators.
*Authors: Daniel San Jose Pro, Oliver Hausdoerfer, Martin Schuck and Angela Schoellig.*

> Collection of C++ controllers for torque-based control for manipulators compatible with `ros2_control`, including Operational Space Control and Cartesian Impedance Control. 

Check the [controllers :simple-github:](https://github.com/utiasDSL/crisp_controllers) , robot [demos :simple-github:](https://github.com/utiasDSL/crisp_controllers_demos), and simple [python interface :simple-github:](https://github.com/utiasDSL/crisp_py).

| ![Franka](media/franka.gif) | ![kinova](media/kinova.gif) | ![iiwa](media/iiwa.gif) |
|:--:|:--:|:--:|
| *Robot following a moving target, while base joint follows a sine curve.* | *Simulated kinova robot with continous joints and nullspace control* | *Another simulated robot example...* |

| ![franka_eight_reduced](media/franka_eight_reduced.gif) ![franka_ns_reduced](media/franka_ns_reduced.gif)  | ![vicon](media/franka_teleop.gif)|
|:--:|:--:|
| *Real robot following a target and being disturbed (contact) + null space control demonstration*  | *Demonstration using a cartesian controller teleoperated using Vicon tracking system (Speed x4)*| 


## Why?

Learning-based controllers, such as diffusion policies, deep reinforcement learning, and foundation models, typically output low-frequency or sporadic target poses, necessitating a low-level controller to track these references smoothly, especially in contact-rich environments.
While `ROS2` frameworks like `MoveIt` offer comprehensive motion planning capabilities, they are often unnecessarily complex for tasks requiring simple, real-time pose or joint servoing.

We present a set of lightweight, torque-based Cartesian and joint-space controllers implemented in C++ for `ros2_control`, compatible with any robot exposing an effort interface—a common standard among modern manipulators.
Our controllers incorporate friction compensation, joint limit avoidance, and error clipping, and have been validated on the Franka Robotics FR3 manipulator.

Designed for fast integration and real-time control, our implementation lowers the barrier to deploying learning-based algorithms on `ROS2`-compatible platforms.
The code is available at [github.com/utiasDSL/crisp_controllers](https://github.com/utiasDSL/crisp_controllers).

## Citing

```bibtex
@inproceeding{
   TODO
}
```
