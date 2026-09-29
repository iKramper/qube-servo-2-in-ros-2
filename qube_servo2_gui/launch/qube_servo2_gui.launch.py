#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    config_file = PathJoinSubstitution([
        FindPackageShare('qube_servo2_gui'),
        'config',
        'gui_config.yaml'
    ])

    hmi = Node(
        package='qube_servo2_gui',
        executable='qube_servo2_gui_node',
        name='qube_servo2_hmi',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': ParameterValue(
                    use_sim_time,
                    value_type=bool
                )
            }
        ]
    )

    return LaunchDescription([
        # ROS 2 parameter YAML uses dot-decimal numeric values. Forcing only
        # LC_NUMERIC avoids the es_CO comma-decimal issue without changing the
        # user's keyboard or desktop language.
        SetEnvironmentVariable(name='LC_NUMERIC', value='C'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock'
        ),

        hmi
    ])
