cmake_minimum_required(VERSION 3.8)

project(qube_servo2_controllers
    VERSION 1.0.0
    DESCRIPTION "Quanser qube-servo2-motor package for controllers logic"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(
        -Wall
        -Wextra
        -Wpedantic
        -Werror=return-type
    )
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)


# Unitary controller:
add_library(qube_servo2_voltage_controller SHARED
    src/qube_voltage_controller.cpp
)

target_include_directories(qube_servo2_voltage_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(qube_servo2_voltage_controller
    controller_interface
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    std_msgs
)

# PID controller:
add_library(qube_servo2_pid_controller SHARED
    src/discrete_pid.cpp
    src/qube_pid_controller.cpp
)


target_include_directories(qube_servo2_pid_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies( qube_servo2_pid_controller
    controller_interface
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    std_msgs
)

# State feedback controller:
add_library(qube_servo2_state_feedback_controller SHARED
    src/qube_state_feedback_controller.cpp
)


target_include_directories(qube_servo2_state_feedback_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


ament_target_dependencies(qube_servo2_state_feedback_controller
    controller_interface
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    std_msgs
)

pluginlib_export_plugin_description_file(
    controller_interface
    qube_servo2_controllers_plugins.xml
)

install(
    TARGETS 
    
    qube_servo2_voltage_controller
    qube_servo2_pid_controller
    qube_servo2_state_feedback_controller

    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(DIRECTORY config launch urdf
    DESTINATION share/${PROJECT_NAME}
)


ament_package()