import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare a launch argument for the interval (default: 1000 ms)
        DeclareLaunchArgument(
            'interval', default_value='1000', description='Interval at which to send status messages'),

        # Launch the sensor_server Python node immediately
        Node(
            package='tcp_examples',
            executable='sensor_server.py',
            name='sensor_server',
            output='screen',
            parameters=[{'interval': LaunchConfiguration('interval')}]
        ),

        # Adding a 2-second delay before launching the sensor_client node
        TimerAction(
            period=2.0,  # 2-second delay
            actions=[
                Node(
                    package='tcp_examples',
                    executable='sensor_client.py',
                    name='sensor_client',
                    output='screen',
                    parameters=[{'interval': LaunchConfiguration('interval')}]
                )
            ]
        ),
    ])
