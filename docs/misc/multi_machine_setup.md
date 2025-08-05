To setup multiple machines, we recommend using a different RMW. Either go with Zenoh or CycloneDDS.
We prefer to use CycloneDDS.

## Using CycloneDDS for multi-machine setups

1. Make sure that the Cyclone RMW is installed `ros-$ROS_DISTRO-rmw-cyclonedds-cpp`. If you use the `pixi.toml` provided
    in this repo it should be the case.


2. Now you can modify your`scripts/set_env.sh` to include further configuration lines: 
    ```bash hl_lines="4-7" title="scripts/set_env.sh"
    export ROS_DOMAIN_ID=100  # (1)! 
    export CRISP_CONFIG_PATH=/path/to/crisp_py/config  # (1)!

    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=file:///path/to/crisp_gym/scripts/cyclone_config.xml  # (2)!

    ros2 daemon stop & ros2 daemon start  # (3)!
    ```

    1. Check the [getting started](../getting_started.md) to see why we set this.
    2. __TODO__: this file as well as the path need to be modified!
    3. The communication daemon needs to be restarted to account for the changes.

3. Update the config for your setup to use the correct network interface:
    ```bash hl_lines="6" title="scripts/cyclone_config.xml"
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


4. Finally, check that everything is working. 
Enter in the humble shell with `pixi shell -e humble` and if your robot is active, run `ros2 topic list` and you should see some topics listed!

## Using Zenoh for multi-machine setups

:construction:

## Troubleshooting
- Run `env | grep RMW_IMPLEMENTATION`, if the variable is not set, you need to make sure that the script `scripts/set_env.sh` is being executed!
- Be sure that the name of the interface is correct!

## References

- iRobot Middleware Config: https://iroboteducation.github.io/create3_docs/setup/xml-config/
- MoveitPro customize DDS: https://docs.picknik.ai/how_to/configuration_tutorials/customize_dds_configuration/

- Cyclone Run-Time-Configuration: https://github.com/eclipse-cyclonedds/cyclonedds/tree/a10ced3c81cc009e7176912190f710331a4d6caf#run-time-configuration
- StereoLabs improve performance: https://www.stereolabs.com/docs/ros2/dds_and_network_tuning#change-dds-middleware
- Husarion DDS setup: https://husarion.com/tutorials/other-tutorials/husarnet-cyclone-dds/

- ROS2 Doctor: https://docs.ros.org/en/kilted/Tutorials/Beginner-Client-Libraries/Getting-Started-With-Ros2doctor.html
