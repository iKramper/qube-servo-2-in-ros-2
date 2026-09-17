#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Joint state broadcaster:
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '30'
        ],
        output='screen'
    )

    # Motor voltage controller:
    motor_voltage_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'motor_voltage_controller',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '30'
        ],
        output='screen'
    )


    return LaunchDescription([
        joint_state_broadcaster,
        motor_voltage_controller
    ])