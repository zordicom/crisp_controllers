# CRISP ros2_controllers
<a href="https://github.com/danielsanjosepro/crisp_controllers/actions/workflows/humble_ros2_ci.yml"><img src="https://github.com/danielsanjosepro/crisp_controllers/actions/workflows/humble_ros2_ci.yml/badge.svg"/></a>
<a href="https://github.com/danielsanjosepro/crisp_controllers/actions/workflows/jazzy_ros2_ci.yml"><img src="https://github.com/danielsanjosepro/crisp_controllers/actions/workflows/jazzy_ros2_ci.yml/badge.svg"/></a>
<a href="https://github.com/danielsanjosepro/crisp_controllers/actions/workflows/rolling_ros2_ci.yml"><img src="https://github.com/danielsanjosepro/crisp_controllers/actions/workflows/rolling_ros2_ci.yml/badge.svg"/></a>
<a href="https://danielsanjosepro.github.io/crisp_controllers/"><img alt="Static Badge" src="https://img.shields.io/badge/docs-passing-blue?style=flat&link=https%3A%2F%2Fdanielsanjosepro.github.io%2Fcrisp_controllers%2F"></a>

Collection of C++ controllers for torque-based control for manipulators in ROS2. Tested on the Franka Robotics FR3 Robot.
Check the [docs](https://danielsanjosepro.github.io/crisp_controllers/)!

| ![Figure 1](https://github.com/user-attachments/assets/5b12bd87-7880-4125-89ba-c3a682a938ff) | ![Figure 2](https://github.com/user-attachments/assets/5b12bd87-7880-4125-89ba-c3a682a938ff) |
|:--:|:--:|
| *Robot following a moving target, while base joint follows a sine curve.* | *TODO: Real robot following a target and being disturbed (contact)* |


## Features

- 🤖 Operational Space Controller as well as Joint Impedance Controller for torque-based control.  
- 🚫 No MoveIt or complicated path-planning, just a simple C++ `ros2_controller`. Ready to use.  
- ⚙️ Dynamically and highly parametrizable: powered by the `generate_parameter_library` you can modify stiffness and more during operation.  
- 🐍 Python interface to move your ROS2 robot around without having to think about topics, spinning, and more ROS2 jargon!


