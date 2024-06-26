import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            LogInfo, RegisterEventHandler)


def generate_launch_description():

    example_node =Node(
        package='crazyflie_examples',
        executable='set_param',
        name='set_param',
        )


    return LaunchDescription([
        DeclareLaunchArgument('server_yaml_file', default_value=''),
        Node(
            package='crazyflie',
            executable='crazyflie_server',
            name='crazyflie_server',
            output='screen',
            parameters= ['swarms/cs2-test-yamls/cs2_server.yaml']
        ),
        example_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=example_node,
                on_exit=[
                    LogInfo(msg=('Param example has ended')),
                    EmitEvent(event=Shutdown(
                        reason='example ended'))
                ]
            )
        )
    ])
