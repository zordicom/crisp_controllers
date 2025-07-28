To setup multiple machines, we recommend using a different RMW. Either go with Zenoh or CycloneDDS.
We prefer to use CycloneDDS.

## Using CycloneDDS for multi-machine setups

1. Make sure that the Cyclone RMW is installed `ros-$ROS_DISTRO-rmw-cyclonedds-cpp`. If you use the `pixi.toml` provided
    in this repo it should be the case. You can check it in the terminal by running  
    ```bash
    ros2 pkg list | grep rmw_cyclonedds_cpp  # (1)!
    ```

    1. this should print the package name if installed.

2. Now you can setup the required environment variables:
    ```bash
    export ROS_DOMAIN_ID=XXX  # (1)! 
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=file:///path/to/cyclone_config.xml  # (2)!
    ```

    1. Same as the robot you want to communicate with
    2. __TODO__: change this!

    You can put this in a script called `scripts/personal_ros_env.sh`, which will be called be `scripts/set_ros_env.sh` every 
    time you open the pixi shell. That script will automatically restart the `ros2 daemon` so that the config is set.

3. Update the config for your setup:
    ```bash hl_lines="6"
    <CycloneDDS xmlns="https://cdds.io/config">
    <Domain>
        <General>
            <AllowMulticast>true</AllowMulticast>
            <Interfaces>
                <NetworkInterface name="enx607d0937fb24" />   # (1)!
            </Interfaces>
        </General>
        <Discovery>
            <ParticipantIndex>auto</ParticipantIndex>
            <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
        </Discovery>
    </Domain>
    </CycloneDDS>

    ```

    1. Modify this with your network interface: check `ip addr` on your shell.


4. Finally, check that everything is working. If your robot is active, run `ros2 topic list` and you should see some topics listed!

## Using Zenoh for multi-machine setups

:construction:

## Troubleshooting
- Run `env | grep RMW_IMPLEMENTATION`, if the variable is not set, you need to make sure that the script `scripts/set_ros_env.sh` is being executed and that your script `scripts/personal_ros_env.sh` is properly defined.
- Be sure that the name of the interface is correct!

