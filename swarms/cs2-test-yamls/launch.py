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
    script = LaunchConfiguration('script')
    backend = LaunchConfiguration('backend')

    script_launch_arg = DeclareLaunchArgument(
        'script'
    )

    backend_launch_arg = DeclareLaunchArgument(
        'backend',
        default_value='cpp'
    )

    crazyflie = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('crazyflie'), 'launch'),
            '/launch.py']),
        launch_arguments={
            'backend': backend,
            }.items()
    )

    example_node = Node(
        package='crazyflie_examples',
        executable=script,
        name=script,
        parameters=[{
            'use_sim_time': PythonExpression(["'", backend, "' == 'sim'"]),
        }]
    )

    event_close =    RegisterEventHandler(
            OnProcessExit(
                target_action=example_node,
                on_exit=[
                    LogInfo(msg=('Param example has ended')),
                    EmitEvent(event=Shutdown(
                        reason='example ended'))
                ]
            )
        )

    return LaunchDescription([
        script_launch_arg,
        backend_launch_arg,
        crazyflie,
        example_node,
        event_close
    ])

def generate_launch_description():

    example_node =Node(
        package='crazyflie_examples',
        executable='set_param',
        name='set_param',
        )


    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='cpp'),
        DeclareLaunchArgument('debug', default_value='False'),
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
