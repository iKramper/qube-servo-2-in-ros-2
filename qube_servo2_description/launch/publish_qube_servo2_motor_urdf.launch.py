#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    xacro_file = PathJoinSubstitution([
        FindPackageShare('qube_servo2_description'),
                'urdf',
                'qube_base.urdf.xacro'
    ])
    
    physical_config_file = PathJoinSubstitution([
        FindPackageShare('qube_servo2_description'),
        'config',
        'qube_servo2_motor_params.yaml'
    ])
    
    robot_description = ParameterValue(
        Command([
                'xacro ', xacro_file,
                ' physical_config_file:=', physical_config_file
            ]),
            value_type = str
    )
    
    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        name = 'qube_servo2_state_publisher',
        parameters = [{
            'robot_description' : robot_description,
            'use_sim_time' : use_sim_time
        }],
        output = 'screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value = 'true',
            description = 'Use gz sim time'
        ),
        robot_state_publisher_node
    ])