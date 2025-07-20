<img src="media/crisp_logo.png" alt="CRISP Controllers Logo"/>

<a href="https://github.com/utiasDSL/crisp_controllers/actions/workflows/humble_ros2_ci.yml"><img src="https://github.com/utiasDSL/crisp_controllers/actions/workflows/humble_ros2_ci.yml/badge.svg"/></a>
<a href="https://github.com/utiasDSL/crisp_controllers/actions/workflows/jazzy_ros2_ci.yml"><img src="https://github.com/utiasDSL/crisp_controllers/actions/workflows/jazzy_ros2_ci.yml/badge.svg"/></a>
<a href="https://github.com/utiasDSL/crisp_controllers/actions/workflows/kilted_ros2_ci.yml"><img src="https://github.com/utiasDSL/crisp_controllers/actions/workflows/kilted_ros2_ci.yml/badge.svg"/></a>
<a href="https://github.com/utiasDSL/crisp_controllers/actions/workflows/rolling_ros2_ci.yml"><img src="https://github.com/utiasDSL/crisp_controllers/actions/workflows/rolling_ros2_ci.yml/badge.svg"/></a>
<a href="https://danielsanjosepro.github.io/crisp_controllers/"><img alt="Static Badge" src="https://img.shields.io/badge/docs-passing-blue?style=flat&link=https%3A%2F%2Fdanielsanjosepro.github.io%2Fcrisp_controllers%2F"></a>

CRISP is a collection of C++ controllers for torque-based control for manipulators compatible with `ros2_control`, including Operational Space Control and Cartesian Impedance Control. Robot agnostic and tested on the Franka Robotics FR3 Robot in single and bimanual operation. Check the [docs](https://danielsanjosepro.github.io/crisp_controllers/)!

## Demos

Some of the things that you can do with our controllers. Check the [crisp_controller_demos](https://github.com/utiasDSL/crisp_controllers_demos) for more information on how to get the controllers running on you robot.

| ![Franka](media/franka.gif) | ![kinova](media/kinova.gif) | ![iiwa](media/iiwa.gif) |
|:--:|:--:|:--:|
| *Robot following a moving target, while base joint follows a sine curve.* | *Simulated kinova robot with continous joints and nullspace control* | *Another simulated robot example...* |

| ![franka_eight_reduced](media/franka_eight_reduced.gif) ![franka_ns_reduced](media/franka_ns_reduced.gif)  | ![vicon](media/franka_teleop.gif)|
|:--:|:--:|
| *Real robot following a target and being disturbed (contact) + null space control demonstration*  | *Demonstration using a cartesian controller teleoperated using Vicon tracking system (Speed x4)*| 

## Features

- 🤖 Operational Space Controller as well as Cartesian Impedance Controller for torque-based control.  
- 🚫 No MoveIt or complicated path-planning, just a simple C++ `ros2_controller`. Ready to use.  
- ⚙️ Dynamically and highly parametrizable: powered by the `generate_parameter_library` you can modify stiffness and more during operation.  
- 🐍 Python interface to move your ROS2 robot around without having to think about topics, spinning, and more ROS2 jargon! Check [crisp_py](https://github.com/utiasDSL/crisp_py) for more information and examples.
- ❓ Demos showcasing how to use the controller with FR3 of Franka Emika in single and bimanual setup. Check the [crisp_controller_demos](https://github.com/utiasDSL/crisp_controllers_demos).

## Getting Started

Independently of your setup, you *should* use a real-time patch. You don't want any jitter with this type of controllers otherwise they can get unstable. Most vendors will not even allow you to use the manipulator.
If you are trying it out on simulation, you should not worry about it. If using a real robot, you can check the [Franka Robotics guide on how to set up a patch](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel). 

### From source

Include it to your ros2 workspace and install dependencies
```bash
cd ~/ros2_ws  # or wherever you ws is...
git clone https://github.com/utiasDSL/crisp_controllers.git src/crisp_controllers
source /opt/ros/$ROS_DISTRO/setup.sh
rosdep update
rosdep install -q --from-paths src --ignore-src -y  # Install missing dependencies
colcon build --packages-select crisp_controllers  # Build the package
touch src/crisp_controllers/COLCON_IGNORE  # Forget about it
```

### Docker
You can try the controller with the provided Dockerfile and devcontainer setup in [crisp_controller_demos](https://github.com/utiasDSL/crisp_controllers_demos). Our docker setup uses the RT-capabilities if available. If you are using VSCode, you should be able
to open the container directly from there using the devcontainer plugin. Check [the devcontainer documentation](https://code.visualstudio.com/docs/devcontainers/containers) for more information.
