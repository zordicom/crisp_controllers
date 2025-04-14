cmake_minimum_required(VERSION 3.5)
project(crisp_controllers)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(CHECK_TIDY "Adds clang-tidy tests" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(
  cartesian_impedance_controller_parameters
  src/cartesian_impedance_controller.yaml
)

generate_parameter_library(
  joint_impedance_controller_parameters
  src/joint_impedance_controller.yaml
)

generate_parameter_library(
  operational_space_controller_parameters
  src/operational_space_controller.yaml
)

generate_parameter_library(
  pose_broadcaster_parameters
  src/pose_broadcaster.yaml
)

add_library(
        ${PROJECT_NAME}
        SHARED
        src/cartesian_impedance_controller.cpp
        src/joint_impedance_controller.cpp
        src/operational_space_controller.cpp
        src/frictionless_controller.cpp
        src/pose_broadcaster.cpp
)


include_directories(
        include
        ${Eigen3_INCLUDE_DIRS}
)
# Link parameter libraries to the main library
target_link_libraries(${PROJECT_NAME}
  cartesian_impedance_controller_parameters
  joint_impedance_controller_parameters
  pose_broadcaster_parameters
)


ament_target_dependencies(
        ${PROJECT_NAME}
        controller_interface
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        pinocchio
        generate_parameter_library
)

pluginlib_export_plugin_description_file(
        controller_interface crisp_controllers.xml)

install(
        TARGETS
        ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

install(
        DIRECTORY include/
        DESTINATION include
)

ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)
ament_export_dependencies(
        controller_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        hardware_interface
)

ament_package()
