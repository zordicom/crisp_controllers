# Getting started


Here is an overview of the CRISP framework (please check our paper for details).

![Stack overview](media/crisp_overview.png#only-light)
![Stack overview](media/crisp_overview_dark.png#only-dark)


- [ ] 1. The first part is the setup for the low-level [crisp_controllers](https://github.com/utiasDSL/crisp_controllers).
- [ ] 2. Then, you will try moving the robot using [CRISP_PY](https://github.com/utiasDSL/crisp_py) python interface.
- [ ] 3. Then, you can optionally include additional cameras and other sensors in your setup. 
- [ ] 4. Finally, you can set up [CRISP_GYM](https://github.com/utiasDSL/crisp_gym) - the Gymnasium interface - and start policy deployment or teleoperation.

![Robots](media/robots_transparent.png)

## 1. Getting the low-level C++ [CRISP](https://github.com/utiasDSL/crisp_controllers) controller ready

The computer running the CRISP controller needs a real-time patch for the controller to run smoothly and safely. You can check out the [Franka Robotics guide on how to set up a RT-patch.](https://frankarobotics.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel)
On newer Ubuntu versions, you can use [Ubuntu Pro](https://ubuntu.com/real-time) for an easy setup.

Then, check if your robot is already included in one of our demos, check [how to run a demo](misc/demos.md) from our [demos repository](https://github.com/utiasDSL/crisp_controllers_demos). You can then follow the instructions there to start your robot(s) using a Docker container. Some of them offer the possibility to run the demos with simulated robots to test the setup.

If your robot is not included in the demos that is not problem. Check out [How to set up a robot that is not available in the demos](misc/new_robot_setup.md). Once you get the controllers running, feel free to open a pull request on our repo to add it to the demos! We highly appreciate that!

## 2. :snake: Use the python interface [CRISP_PY](https://github.com/utiasDSL/crisp_py) to control the robot

### Installation

!!! Note
    If you want to use the gymnasium interface, CRISP_PY will be autonmatically installed in the gym. You can therefore check the installation of [CRISP_GYM](#4-getting-started-with-crisp_gym) directly.
    However, this section still gives you an idea on how to use CRISP_PY with your robot. We do not recommend to skip it.

To use `CRISP_PY`, we recommend using [pixi](https://pixi.sh/latest/) as a package manager, a modern conda-like package manager.
It can be used in combination with [robostack](https://robostack.github.io/) to easily install ROS2 in any machine.
There are a few ways to get you started:

_... install from source:_

```bash
git clone https://github.com/utiasDSL/crisp_py
cd crisp_py
pixi install
pixi shell -e humble
python -c "import crisp_py"  # (1)!
```

1. This should not log anything if everything is fine

_... use in your already existing pixi project:_

To use `CRISP_PY` in an already existing pixi project, you need to make sure that `ROS2` is available.
Check the [pixi.toml](https://github.com/utiasDSL/crisp_py/blob/main/pixi.toml) of `CRISP_PY` to see how this looks like.
Then you can add `CRISP_PY` as a pypi package:
```bash
pixi add --pypi crisp-python
```
or
```bash
uv add crisp-python
```
or
```bash
pip install crisp-python
```
Double-check that everything is working by running:

```bash
python -c "import crisp_py"  # (1)!
```

1. This should not log anything if everything is fine

Now you can try to control the robot! Check out the [examples](https://github.com/utiasDSL/crisp_py/blob/main/examples) for inspiration.

### Try it out with the robot

Make sure that the demo container is running in the background, as we will need it to access the robot.
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
    3. Set target 10 cm downwards. Careful not to send poses that are too far away from the current one!
    4. This will request the controller manager to activate the cartesian impedance controller. You can use it with other controllers like the operational space controller!

## 3. Adding cameras, grippers, and further sensors to CRISP_PY

### Cameras

To add a camera, you will need to run it in a separate container as well.
The cameras that we tested are:

- [Real Sense](https://github.com/IntelRealSense/realsense-ros/tree/ros2-master) which gives amazing ROS2 support,
- and [Orbbec](https://github.com/orbbec/OrbbecSDK_ROS2).

But any camera with [camera_ros](https://github.com/christianrauch/camera_ros) should work.
Check the [demos](misc/demos.md) to see some examples with cameras

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

For gripper control, you need to make sure that a ROS2 node is running that accepts commands through a topic and publishes the state of the gripper.
To use a:

- Franka Hand, you just need to start the demo. An [adapter](https://github.com/utiasDSL/crisp_controllers_demos/blob/main/crisp_controllers_robot_demos/crisp_controllers_robot_demos/crisp_py_franka_hand_adapter.py) is already running to allow you to control the gripper this way,
- Dynamixel motor to control a gripper, we used the well-maintained [dynamixel_hardware_interface](https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface) with a position controller for the gripper.

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


## 4. Getting started with [CRISP_GYM](https://github.com/utiasDSL/crisp_gym)

Similar to `CRISP_PY`, we recommend using `pixi` to install `CRISP_GYM`.

```sh
git clone https://github.com/utiasDSL/crisp_gym
cd crisp_gym
```
Now, you should set a few things before installing everything.
Create a file `scripts/set_env.sh` which will be sourced every time that you run a command in your environment.
The script will not be tracked by git.
In this script you need to add a environment variables:

- `ROS_DOMAIN_ID`: which is used to define nodes that should be able to see each other. In our [demos](misc/demos.md) they are set to 100 as default.
- `CRISP_CONFIG_PATH`: which should be the path to a config folder similar to [config path of CRISP_PY](https://github.com/utiasDSL/crisp_py/tree/main/config). The easiest way to do this is to clone CRISP_PY somewhere
and set this environment variable to point to it.

```sh title="scripts/set_env.sh" hl_lines="2"
export ROS_DOMAIN_ID=100
export CRISP_CONFIG_PATH=/path/to/crisp_py/config  # (1)!
```

1. Modify this!

If you want to work in a multi-machine setup (e.g. policy runs in a different machine as controllers), then check [how to setup multi-machine in ROS2](misc/multi_machine_setup.md).


```sh
source scripts/configure.sh  # (1)!
pixi install
pixi shell -e humble-lerobot
python -c "import crisp_gym"

```

1. This will set some environment variable pre-installation as well as checking that you defined the previous script properly.

If the previous steps worked, then you are good to go.

### Teleoperation: Record data in [LeRobotFormat](https://github.com/huggingface/lerobot)

You can record data in `LeRobotFormat` to train a policy directly in [LeRobot](https://github.com/huggingface/lerobot).
You will need to use teleoperation to record data and we highly recommend using a leader-follower setup to generate episodes. 

#### Leader-follower

The leader can be controlled by a human operator and the follower will mimic its motion.
Checkout `scripts/leader_follower_teleop.py` to get an idea on how the code works.
For your specific setup you need to:

- Define your own `TeleopRobotConfig`, check [`teleop_robot_config.py`](https://github.com/utiasDSL/crisp_gym/blob/main/crisp_gym/teleop/teleop_robot_config.py).
- Define your own `ManipulatorEnvConfig`, check [`manipulator_env_config.py`](https://github.com/utiasDSL/crisp_gym/blob/main/crisp_gym/manipulator_env_config.py).

Then, to record data use:
```sh
pixi run -e humble-lerobot python scripts/record_lerobot_format_leader_follower.py \
   --repo-id <your_account>/<repo_name> # (1)!
```

1. Add `--help` to check other parameters to pass to the record function.

The script is interactive. It will first ask to choose the desired configuration files for the recording and then allow you to record episodes interactively.
There are two recording methods currently available:

- `keyboard` (default): It allows you to record episodes using the keyboard with the keys 
    - __r__(ecord start/stop) an episode,
    - __d__(elete episode) after recording a failed episode,
    - __s__(ave episode) after recording a succesful episode,
    - __q__(uit) after finishing.
- `ros`: It uses the topic `recording_state` to catch `String` ROS2 messages to follow the same recording workflow as the keyboard. 
    With this you can implement custom recording devices to control the recording workflow

    ??? example "Using the FR3 pilot buttons of Franka Robotics as a recording device"
        In our lab, we use the buttons of the leader robot as a recording device with the [franka-buttons](https://github.com/nakama-lab/franka-buttons) repository.
        The following script uses the circle, cross, check and up buttons as a record, delete, save and quit commands respectively:
        ```py
        """Send recording commands for an episode recorder node to start, stop recording, save episodes and quit using the franka pulot buttons."""
        import rclpy
        from rclpy.node import Node

        from franka_buttons_interfaces.msg import FrankaPilotButtonEvent
        from std_msgs.msg import String


        class ButtonToRecordMessage(Node):
            """Node that subscribes to the button event and toggles the gripper when the circle button is pressed."""

            def __init__(self) -> None:
                super().__init__("button_to_record_message")

                self.create_subscription(
                    FrankaPilotButtonEvent, "franka_pilot_button_event", self.button_callback, 10
                )

                self.publisher = self.create_publisher(String, "record_transition", 10)

                # Add a cooldown to avoid multiple toggles
                self._last_toggle = self.get_clock().now()
                self._cooldown = 0.5

                self.get_logger().info("ButtonToRecordMessage node started.")

            def button_callback(self, msg: FrankaPilotButtonEvent):
                """Callback function for the button event.

                If circle pressed, then pass the command to the gripper client to toggle the gripper.
                """
                if (self.get_clock().now() - self._last_toggle).nanoseconds < self._cooldown * 1e9:
                    return

                if msg.pressed:
                    if msg.pressed[0] == "circle":
                        self.get_logger().info("Circle button pressed. Sending a record message.")
                        self.publisher.publish(String(data="record"))
                    if msg.pressed[0] == "check":
                        self.get_logger().info("Check button pressed. Sending a save episode message.")
                        self.publisher.publish(String(data="save"))
                    if msg.pressed[0] == "cross":
                        self.get_logger().info("Cross button pressed. Sending a delete episode message.")
                        self.publisher.publish(String(data="delete"))
                    if msg.pressed[0] == "up":
                        self.get_logger().info("UP button pressed. Sending a quit command message.")
                        self.publisher.publish(String(data="exit"))

                    self._last_toggle = self.get_clock().now()


        def main():
            rclpy.init()
            node = ButtonToRecordMessage()
            rclpy.spin(node)
            rclpy.shutdown()


        if __name__ == "__main__":
            main()
        ```

After this, you can visualize the episodes with rerun visualizer and LeRobot utils:
```sh
pixi run -e lerobot python -m lerobot.scripts.visualize_dataset \
        --repo-id <your_account>/<repo_name> \
        --episode-index 0
```
...or use the [online tool for visualization](https://huggingface.co/spaces/lerobot/visualize_dataset).

!!! warning
    LeRobot is subjected to rapid changes. This command might change in future versions.

#### Other teleop setups

You can add further teleop options to [`teleop/`](https://github.com/utiasDSL/crisp_gym/blob/main/crisp_gym/teleop) and create 
a similar record script as [`scripts/record_lerobot_format_leader_follower.py`](https://github.com/utiasDSL/crisp_gym/blob/main/crisp_gym/scripts/record_lerobot_format_leader_follower.py)

### Train a policy

You can use LeRobot train scripts to train a policy simply by running:
```sh
pixi run -e lerobot python -m lerobot.scripts.train \
          --dataset.repo_id=<your_account>/<repo_name> \
          --policy.type=diffusion \
          --policy.push_to_hub=false
```

!!! warning
    LeRobot is subjected to rapid changes. This command might change in future versions.

They provide the latest implementations of most VLA.
Check [LeRobot](https://github.com/huggingface/lerobot) for more information.

### Deploy policy

After training with LeRobot, you can deploy the policy with:
```sh
pixi run -e humble-lerobot python scripts/deploy_policy.py # (1)!
```

1. The script will interactively allow you to choose a model inside `outputs/train`. If you want to explicitly pass a path you can override it with `--path`

!!! warning
    LeRobot is subjected to rapid changes. This command might change in future versions.

Good job, now you can evaluate your model!

