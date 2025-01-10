from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction  # Added TimerAction import
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node  # Correct import for Node

def generate_launch_description():
    return LaunchDescription([
        # Declare the 'interval' launch argument with a default value of 1000
        DeclareLaunchArgument('interval', default_value='1000', description='Interval in ms for sensor_client'),
        
        # Launch the sensor_server node immediately
        Node(
            package='tcp_examples',
            executable='sensor_server',
            name='sensor_server_node',
            output='screen'
        ),
        
        # Add a 2-second delay before launching the sensor_client node for proper execution
        TimerAction(
            period=2.0,  # Delay in seconds
            actions=[
                Node(
                    package='tcp_examples',
                    executable='sensor_client',
                    name='sensor_client_node',
                    output='screen',
                    parameters=[{'interval': LaunchConfiguration('interval')}]
                )
            ]
        )
    ])
