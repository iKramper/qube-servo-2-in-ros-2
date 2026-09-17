#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Launch arguments:
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Find packages:
    pkg_gz = FindPackageShare(
        'qube_servo2_gz'
    )

    pkg_controllers = FindPackageShare(
        'qube_servo2_controllers'
    )

    # Launch files:
    gazebo_launch = PathJoinSubstitution([
        pkg_gz,
        'launch',
        'spawn_world.launch.py'
    ])

    urdf_launch = PathJoinSubstitution([
        pkg_controllers,
        'launch',
        'publish_qube_servo2_controlled_urdf.launch.py'
    ])

    spawn_launch = PathJoinSubstitution([
        pkg_gz,
        'launch',
        'spawn_robot.launch.py'
    ])

    controllers_launch = PathJoinSubstitution([
        pkg_controllers,
        'launch',
        'spawn_controllers.launch.py'
    ])

    spawn_models_launch = PathJoinSubstitution([
        pkg_gz,
        'launch',
        'spawn_models.launch.py'
    ])

    spawn_gazebo_bridge = PathJoinSubstitution([
        pkg_gz,
        'launch',
        'gz_bridge.launch.py'
    ])


    return LaunchDescription([

        use_sim_time_arg,

        # 1. Start Gazebo:
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                gazebo_launch
            ),
            launch_arguments={
                'use_sim_time': use_sim_time
            }.items()
        ),
        
        # 2. Start Gazebo - ROS 2 bridge:
        TimerAction(
            period=0.5,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        spawn_gazebo_bridge
                    )
                )
            ]
        ),

        # 2. Publish complete controlled URDF:
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        urdf_launch
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time
                    }.items()
                )
            ]
        ),

        # 3. Spawn robot in Gazebo:
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        spawn_launch
                    )
                )
            ]
        ),

        # 4. Spawn ROS 2 controllers:
        TimerAction(
            period=6.5,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        controllers_launch
                    )
                )
            ]
        )
    ])