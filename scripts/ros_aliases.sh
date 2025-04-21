# Some aliases to quickly get compile source and navigate to project
alias proj="cd ~/ros2_ws/src/crisp_controllers"
alias cb="cd ~/ros2_ws && colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && proj"
alias sw="source /opt/ros/$ROS_DISTRO/setup.bash && source ~/ros2_ws/install/setup.bash"
