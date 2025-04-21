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

## Getting Started

Independently of your setup, you *should* use a real-time patch. You don't want any jitter with this type of controllers otherwise they can get unstable.
If you are trying it out on simulation, you should not worry about it. If using a real robot, you can check the [Franka Robotics guide on how to set up a patch](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel). 

### From source

Include it to your ros2 workspace and install dependencies
```bash
cd ~/ros2_ws  # or wherever you ws is...
git clone https://github.com/danielsanjosepro/crisp_controllers.git -b main
rosdep update
rosdep install -q --from-paths src --ignore-src -y  # Install missing dependencies (pinocchio...)
colcon build --packages-select crisp_controllers  # Build the package
touch src/crips_controllers/COLCON_IGNORE  # Forget about it
```

### Docker
You can try the controller with the provided Dockerfile using devcontainers. Our docker setup uses the RT-capabilities if available. If you are using VSCode, you should be able
to open the container directly from there using the devcontainer plugin. Check [the devcontainer documentation](https://code.visualstudio.com/docs/devcontainers/containers) for more information.
