
1. First make sure that your ROS2 drivers for the robot you want to use allow direct-torque control. If this is the case, you are good to go. 
Otherwise, you might need to use [cartesian_controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers) for position controlled robots.

1. Install the ROS2 drivers on your computer. We recommend creating a docker container or working in a devcontainer. You can check the [demos](https://github.com/utiasDLS/crisp_controllers_demos) for inspiration.
3. Add the [controllers](https://github.com/utiasDSL/crisp_controllers) to your src folder:
```bash
cd ~/ros2_ws  # or wherever you ws is...
git clone https://github.com/utiasDSL/crisp_controllers.git src/crisp_controllers
source /opt/ros/$ROS_DISTRO/setup.sh
rosdep update
rosdep install -q --from-paths src --ignore-src -y  # Install missing dependencies
colcon build --packages-select crisp_controllers  # Build the package
touch src/crisp_controllers/COLCON_IGNORE  # Forget about it
```
4. Add the controllers to the config file (check out the [FR3 config](https://github.com/utiasDSL/crisp_controllers_demos/blob/main/crisp_controllers_robot_demos/config/fr3/controllers.yaml) for inspiration).

    ??? example "How to add the configuration to the config file"

        ```yaml

        /**:
        controller_manager:
            ros__parameters:
            update_rate: 1000  # Hz

            pose_broadcaster:
                type: crisp_controllers/PoseBroadcaster

            gravity_compensation:
                type: crisp_controllers/CartesianImpedanceController

            cartesian_impedance_controller:
                type: crisp_controllers/CartesianImpedanceController

            joint_impedance_controller:
                type: crisp_controllers/CartesianImpedanceController

            # more controllers...
        /**:
        pose_broadcaster:
            ros__parameters:
            joints:
                - <TODO>

            end_effector_frame: <TODO>
            base_frame: base

        gravity_compensation:
            ros__parameters:
            joints:
                <TODO>

            end_effector_frame: <TODO>
            base_frame: base

            task:
                k_pos_x: 0.0
                k_pos_y: 0.0
                k_pos_z: 0.0
                k_rot_x: 30.0
                k_rot_y: 30.0
                k_rot_z: 30.0

            nullspace: 
                stiffness: 0.0

            use_friction: true
            use_coriolis_compensation: true
            use_local_jacobian: true

        joint_impedance_controller:
            ros__parameters:
            joints:
                <TODO>

            end_effector_frame: <TODO>
            base_frame: base

            task:
                k_pos_x: 0.0
                k_pos_y: 0.0
                k_pos_z: 0.0
                k_rot_x: 0.0
                k_rot_y: 0.0
                k_rot_z: 0.0

            max_delta_tau: 0.5

            nullspace: 
                stiffness: 5.0
                projector_type: none  # So we are directly controlling the joints!
                damping: 0.5
                max_tau: 5.0
                regularization: 1.0e-06
                weights:
                <TODO>

            use_friction: true
            use_coriolis_compensation: true
            use_local_jacobian: true
            limit_error: true
            limit_torques: true


        cartesian_impedance_controller:
            ros__parameters:
            joints:
                - <TODO>

            end_effector_frame: <TODO>
            base_frame: <TODO>

            task:
                k_pos_x: 400.0
                k_pos_y: 400.0
                k_pos_z: 400.0
                k_rot_x: 30.0
                k_rot_y: 30.0
                k_rot_z: 30.0

            nullspace: 
                stiffness: 2.0

            use_friction: false
            use_coriolis_compensation: true
            use_local_jacobian: true
        ```

5. Finally add them to your launch file. (check out [FR3 launch file](https://github.com/utiasDSL/crisp_controllers_demos/blob/main/crisp_controllers_robot_demos/launch/franka.launch.py) for inspiration).

    ??? example "How to add the controllers to the launch file"
        Add the following nodes to the launch description:

        ```py
        ...
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["cartesian_impedance_controller", "--inactive"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_impedance_controller", "--inactive"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["gravity_compensation", "--inactive"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["pose_broadcaster"],
            output="screen",
        ),
        ...
        ```
