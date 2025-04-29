# CRISP ros2_controllers
<a href="https://github.com/danielsanjosepro/crisp_controllers/actions/workflows/humble_ros2_ci.yml"><img src="https://github.com/danielsanjosepro/crisp_controllers/actions/workflows/humble_ros2_ci.yml/badge.svg"/></a>
<a href="https://github.com/danielsanjosepro/crisp_controllers/actions/workflows/jazzy_ros2_ci.yml"><img src="https://github.com/danielsanjosepro/crisp_controllers/actions/workflows/jazzy_ros2_ci.yml/badge.svg"/></a>
<a href="https://github.com/danielsanjosepro/crisp_controllers/actions/workflows/rolling_ros2_ci.yml"><img src="https://github.com/danielsanjosepro/crisp_controllers/actions/workflows/rolling_ros2_ci.yml/badge.svg"/></a>
<a href="https://danielsanjosepro.github.io/crisp_controllers/"><img alt="Static Badge" src="https://img.shields.io/badge/docs-passing-blue?style=flat&link=https%3A%2F%2Fdanielsanjosepro.github.io%2Fcrisp_controllers%2F"></a>

Collection of C++ controllers for torque-based control for manipulators in ROS2. Tested on the Franka Robotics FR3 Robot.
Check the [docs](https://danielsanjosepro.github.io/crisp_controllers/)!

| ![Figure 1](https://github.com/user-attachments/assets/5b12bd87-7880-4125-89ba-c3a682a938ff) | ![kinova](https://github.com/user-attachments/assets/18b0dda6-c9aa-4753-ac5b-004e64d3c9a3) | ![iiwa](https://github.com/user-attachments/assets/5753ab66-e2c3-4102-b32b-419497567ac1) |
|:--:|:--:|:--:|
| *Robot following a moving target, while base joint follows a sine curve.* | *Simulated kinova robot with continous joints and nullspace control* | *Another simulated robot example...* |

| ![franka_eight_reduced](https://github.com/user-attachments/assets/5a53a4c4-4679-4ae0-a12d-7dbcd0dbedb3) ![franka_ns_reduced](https://github.com/user-attachments/assets/65fdf8af-101d-4b04-8388-6f933328cd8c)  | ![vicon](https://github.com/user-attachments/assets/d64c1416-12f0-44ee-821f-e91d0bf6434d)|
|:--:|:--:|
| *Real robot following a target and being disturbed (contact) + null space control demonstration*  | *Demonstration using a cartesian controller teleoperated using Vicon tracking system (Speed x4)*| 

## Features

- 🤖 Operational Space Controller as well as Joint Impedance Controller for torque-based control.  
- 🚫 No MoveIt or complicated path-planning, just a simple C++ `ros2_controller`. Ready to use.  
- ⚙️ Dynamically and highly parametrizable: powered by the `generate_parameter_library` you can modify stiffness and more during operation.  
- 🐍 Python interface to move your ROS2 robot around without having to think about topics, spinning, and more ROS2 jargon! Check [crisp_py](https://github.com/danielsanjosepro/crisp_py) for more information and examples.
- ❓ Demos showcasing how to use the controller with FR3 of Franka Emika in single and bimanual setup. Check the [crisp_controller_demos](https://github.com/danielsanjosepro/crisp_controllers_demos).

## Getting Started

Independently of your setup, you *should* use a real-time patch. You don't want any jitter with this type of controllers otherwise they can get unstable.
If you are trying it out on simulation, you should not worry about it. If using a real robot, you can check the [Franka Robotics guide on how to set up a patch](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel). 

### From source

Include it to your ros2 workspace and install dependencies
```bash
cd ~/ros2_ws  # or wherever you ws is...
git clone https://github.com/danielsanjosepro/crisp_controllers.git src/crisp_controllers
source /opt/ros/$ROS_DISTRO/setup.sh
rosdep update
rosdep install -q --from-paths src --ignore-src -y  # Install missing dependencies
colcon build --packages-select crisp_controllers  # Build the package
touch src/crisp_controllers/COLCON_IGNORE  # Forget about it
```

### Docker
You can try the controller with the provided Dockerfile using devcontainers. Our docker setup uses the RT-capabilities if available. If you are using VSCode, you should be able
to open the container directly from there using the devcontainer plugin. Check [the devcontainer documentation](https://code.visualstudio.com/docs/devcontainers/containers) for more information.
