from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('interval', default_value='1000', description='Interval in ms for start command'),
        
        Node(
            package='your_package_name',
            executable='sensor_client',
            name='sensor_client_node',
            output='screen',
            parameters=[{'interval': LaunchConfiguration('interval')}]
        ),
        
        Node(
            package='your_package_name',
            executable='sensor_server',
            name='sensor_server_node',
            output='screen'
        ),
    ])
