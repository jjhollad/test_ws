from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace='',
            output='screen',
            parameters=[{
                'yaml_filename': '/home/user/sweepbot_ws/map/dillman2.yaml'
            }],
        )
    ])
