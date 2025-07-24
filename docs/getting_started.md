# Getting started

Here is an overview of the framework

![Stack overview](media/crisp_overview.png#only-light)
![Stack overview](media/crisp_overview_dark.png#only-dark)

In short you have:

- [ ] 1. The first part is the setup for the low level controllers i.e. [crisp_controllers](https://github.com/utiasDSL/crisp_controllers).
- [ ] 2. Then, we will try out to move the robot around using [crisp_py](https://github.com/utiasDSL/crisp_py).
- [ ] 3. After that, we can include cameras to the setup or further sensors. 
- [ ] 4. Finally, we can setup [crisp_gym](https://github.com/utiasDSL/crisp_gym) and start recording episodes.

## 1. Getting the low-level controller ready

First, the computer running the controller needs a realtime patch for the controller to run smoothly and safely.
You can check out the [Franka Robotics guide on how to set up a RT-patch.](https://frankarobotics.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel)
On newer ubuntu versions, you can use [Ubuntu Pro](https://ubuntu.com/real-time) for an easy setup.

Then, check if your robot is part of our [demos](https://github.com/utiasDSL/crisp_controllers_demos).
You can then follow the instructions there to start your robot(s) using a docker container.
Some of them offer the possibility to run the demos with a simulated robots to test the setup.

If your robot is not included in the demos, check out [How to setup a robot that is not available in the demos](new_robot_setup.md).
If you get the controllers running, feel free to open a pull-request to add it to the demos!

## 2. Use crisp_py to control the robot

### Installation

To use `crisp_py`, we recommend using [pixi](https://pixi.sh/latest/) as a package manager, a modern conda-like package manager.
It can be used in combination of [robostack](https://robostack.github.io/) to easily install ROS2 in any machine.
There are a few ways to get you started:

_... install from source:_

```bash
git clone https://github.com/utiasDSL/crisp_py
pixi install
pixi shell -e humble
python -c "import crisp_py"  # (1)!
```

1. This should not log anything if everything is fine

_... use in your already existing pixi project:_

To use `crisp_py` in an already existing pixi project, you need to make sure that `ROS2` is available.
Check the [pixi.toml](https://github.com/utiasDSL/crisp_py/blob/main/pixi.toml) of `crisp_py` to see how this looks like.
Then you can add `crisp_py` as a pypi package:
```bash
pixi add --pypi crisp-py
```
or
```bash
uv add crisp-py
```
or
```bash
pip install crisp-py
```
Double-check that everything is working by running:

```bash
python -c "import crisp_py"  # (1)!
```

1. This should not log anything if everything is fine

Now you can try to control the robot! Chech out the [examples](https://github.com/utiasDSL/crisp_py/blob/main/examples) for inspiration.

### Try it out with the robot

Make sure that the demo container is running in the background. We will need it to access it.
From now on, you can instantiate `Robot` objects to control the robot.

??? example "Example robot usage:"
    ```py
    from crisp_py.robot import Robot
    from crisp_py.robot_config import RobotConfig

    robot_config = RobotConfig(...)
    robot = Robot(namespace="...", config=robot_config)  # (1)!
    robot.wait_until_ready()  # (2)!

    print(robot.end_effector_pose)


    robot.controller_switcher_client.switch_controller(
        "cartesian_impedance_controller",  # (4)!
    )  
    x, y, z = robot.end_effector_pose.position
    robot.set_target(position=[x, y, z-0.1])  # (3)!

    robot.shutdown()
    ```

    1. This will get information from the robot asynchronously
    2. Make sure that we get information from the robot before trying to set targets or reading the pose of the robot.
    3. Set target 10 cm downwoards. Careful not to send poses that are too far away from the current one!
    4. This will request the controller manager to activate the cartesian impedance controller. You can use it with other controllers like the operational space controller!

## 3. Adding cameras, grippers, and further sensors

### Cameras

To add a camera, you will need to run it in a container of the separate container as well.
The cameras that we tried are:

- [Real Sense](https://github.com/IntelRealSense/realsense-ros/tree/ros2-master) which give an amazing ROS2 support,
- or [Orbbec](https://github.com/orbbec/OrbbecSDK_ROS2).

But any camera should work with [camera_ros](https://github.com/christianrauch/camera_ros).

??? example "Example camera usage:"
    ```py
    import cv2
    from crisp_py.camera import Camera, CameraConfig

    camera_config = CameraConfig(
        camera_name="primary",
        resolution=(256, 256),  # (1)!
        camera_color_image_topic="camera_name/color/image_raw",  # (2)!
        camera_color_info_topic="camera_name/color/camera_info",
    )

    camera = Camera(config=camera_config)  # (3)!
    camera.wait_until_ready() # (4)!

    cv2.imshow("Camera Image", camera.current_image)  # (5)!
    cv2.waitKey(0)

    ```

    1. You can define a custom resolution, independently of the resolution of the published image.
    2. Set here the topic of your custom camera name
    3. You can also pass `namespace="..."` to give the camera a namespace. This is required for a bimanual setup.
    4. Make sure that we received an image. This will fail with a timeout if the topic is wrong or the camera is not publishing.
    5. This will show you the latest received image!

### Grippers

For the gripper control you need to make sure that a ROS2 node is running that accepts commands through a topic and publishes the state of the gripper.
To use a ...
- Franka Hand you just need to start the demo. An adapter is already running to allow you to control the gripper this way,
- Dynamixel motor to control a gripper we used the well-mantained [dynamixel_hardware_interface](https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface) with a position controller for the gripper.

??? example "Example gripper usage:"
    You can use the gripper in `crisp_py` with:
    ```py
    from crisp_py.gripper import Gripper, GripperConfig

    # config = GripperConfig.from_yaml(path="...")  (1)
    config = GripperConfig(
        min_value=0.0,
        max_value=1.0,
        command_topic="gripper_position_controller/commands",
        joint_state_topic="joint_states",
    )  # (2)!
    gripper = Gripper(gripper_config=config)  # (3)!
    gripper.wait_until_ready()  # (4)!

    print(gripper.value)

    gripper.open()
    # gripper.close()
    # gripper.set_target(0.5)
    ```

    1. You can load the configs from a yaml file. If you calibrate the gripper manually (check the crisp_py docs for more information) you can select this way your custom calibration file.
    2. Set the range of allowed commands (min stands for fully closed, max to fully open) and the topics for the gripper. You can check the topics using `ros2 topic list`
    3. You can also pass `namespace="..."` to give the gripper a namespace. This is required for a bimanual setup.
    4. Make sure that we received a gripper value. This will fail with a timeout if the topic is wrong or the gripper is not publishing.

### Sensors

You can add further sensors (Force Torque Sensor, Tactile Sensor...) by adding a custom `Sensor` that subscribes to a topic.
Check the examples for more information.


## 4. Using the Gym

Similar to `crisp_py`, we recommend using `pixi` to install `crisp_gym`.

```sh
git clone https://github.com/utiasDSL/crisp_gym
pixi install
pixi shell -e humble
python -c "import crisp_gym"
```

### Record data in LeRobotFormat

You can record data in `LeRobotFormat` to train a policy directly in [lerobot](https://github.com/huggingface/lerobot) by runing:
TODO: Add installation steps for lerobot.

```sh
pixi run -e humble-lerobot python scripts/record_data.py
```

After recording the data, you can use the dataset to train a policy with [lerobot](https://github.com/huggingface/lerobot).
They provided the latest implementations

### Deploy policy

...with `lerobot`:
```sh
pixi run -e humble-lerobot python scripts/deploy_policy.py
```

...with a custom ONNX model:
TODO








