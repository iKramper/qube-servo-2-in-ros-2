cmake_minimum_required(VERSION 3.16)

project(qube_servo2_gui
    VERSION 1.1.0
    DESCRIPTION "Qt6 HMI for Quanser QUBE-Servo 2 on ROS 2"
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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Charts)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(GUI_HEADERS
    include/qube_servo2_gui/app/main_window.hpp
    include/qube_servo2_gui/core/models/telemetry_sample.hpp
    include/qube_servo2_gui/core/models/waveform_config.hpp
    include/qube_servo2_gui/core/services/waveform_generator.hpp
    include/qube_servo2_gui/core/logging/data_recorder.hpp
    include/qube_servo2_gui/infra/ros/qube_ros_bridge.hpp
    include/qube_servo2_gui/ui/style/theme_manager.hpp
    include/qube_servo2_gui/ui/widgets/value_tile.hpp
    include/qube_servo2_gui/ui/widgets/scope_plot.hpp
    include/qube_servo2_gui/ui/pages/dashboard_page.hpp
    include/qube_servo2_gui/ui/pages/control_page.hpp
    include/qube_servo2_gui/ui/pages/recording_dialog.hpp
    include/qube_servo2_gui/ui/pages/system_page.hpp
)

set(GUI_SOURCES
    src/main.cpp
    src/app/main_window.cpp
    src/core/telemetry_sample.cpp
    src/core/services/waveform_generator.cpp
    src/core/logging/data_recorder.cpp
    src/infra/ros/qube_ros_bridge.cpp
    src/ui/style/theme_manager.cpp
    src/ui/widgets/value_tile.cpp
    src/ui/widgets/scope_plot.cpp
    src/ui/pages/dashboard_page.cpp
    src/ui/pages/control_page.cpp
    src/ui/pages/recording_dialog.cpp
    src/ui/pages/system_page.cpp
)

qt_add_resources(qube_servo2_gui_RCC
    resources/resources.qrc
)

add_executable(${PROJECT_NAME}_node
    ${GUI_SOURCES}
    ${GUI_HEADERS}
    ${qube_servo2_gui_RCC}
)

target_include_directories(${PROJECT_NAME}_node
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
    std_msgs
    sensor_msgs
    control_msgs
)

target_link_libraries(${PROJECT_NAME}_node
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::Charts
)

install(TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    config
    resources
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

ament_package()
