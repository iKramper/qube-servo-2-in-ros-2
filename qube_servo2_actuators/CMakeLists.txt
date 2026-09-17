cmake_minimum_required(VERSION 3.8)

project(qube_servo2_actuators
    VERSION 1.0.0
    DESCRIPTION "Quanser qube-servo2-motor package for actuators physical description"
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
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(qube_servo2_motor_model SHARED 
    src/dc_motor_model.cpp
)

target_include_directories(qube_servo2_motor_model PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_library(qube_servo2_gz_system SHARED
    src/qube_servo2_gz_system.cpp
)

target_include_directories(qube_servo2_gz_system PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(qube_servo2_gz_system PUBLIC
    qube_servo2_motor_model
    gz-sim::gz-sim
    gz_ros2_control::gz_ros2_control-system
    hardware_interface::hardware_interface
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(gz_ros2_control
    qube_servo2_actuators_plugins.xml
)

install(
    TARGETS
        qube_servo2_motor_model
        qube_servo2_gz_system
    EXPORT export_qube_servo2_actuators
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY include/
    DESTINATION include
)

ament_export_targets(
    export_qube_servo2_actuators
    HAS_LIBRARY_TARGET
)

ament_export_dependencies(
    gz_ros2_control
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
)

ament_package()