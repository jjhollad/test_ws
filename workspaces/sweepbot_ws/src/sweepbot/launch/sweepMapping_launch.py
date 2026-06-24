import os

from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def launch_setup(context, *args, **kwargs):

    create_bringup_dir = os.path.join(get_package_share_path('create_bringup'), 'launch')
    sensors_dir = os.path.join(get_package_share_path('create3_lidar_slam'), 'launch')
    slam_toolbox_dir = os.path.join(get_package_share_path('create3_lidar_slam'), 'launch')
    world = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'worlds', 'turtlebot3_world.world')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    nav2_file_dir = get_package_share_directory('turtlebot3_navigation2')
    gazebo_launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    cartographer_launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_cartographer'), 'launch')
    sweepbot_dir = os.path.join(get_package_share_path('sweepbot'), 'config')

#   use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')

    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    print(param_file_name)
  
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch_file_dir, 'robot_state_publisher.launch.py')
        ),
#        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    cartographer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cartographer_launch_file_dir, 'cartographer.launch.py')
        ),
#        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_file_dir, 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={
#       'map': os.path.join(nav2_file_dir, 'map', 'map.yaml'),
# 	    'map': os.path.join(os.path.expanduser('~'), 'sweepbot_ws', 'map', 'dillman2.yaml'),
# 	    'use_sim_time': use_sim_time,
            'params_file': os.path.join(sweepbot_dir, 'nav2_params.yaml')
            }.items()
    )
    demo_cmd = Node(
        package='sweepbot',
        executable='demo_sweep',
        emulate_tty=True,
        output='screen',
    )
    create_bringup_cmd = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(create_bringup_dir, 'create_2.launch')
        )
    )
#********"create_bringup create_2.launch" does not have the ending .py???
    sensors_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sensors_dir, 'sensors_launch.py')
        )
    )
    slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'slam_toolbox_launch.py')
        )
    )

    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': os.path.join(os.path.expanduser('~'), 'sweepbot_ws', 'map', 'dillman2.yaml')
        }]
    )

    amcl_cmd = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_map_topic': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'scan_topic': '/scan',
            'laser_model_type': 'likelihood_field',
            'update_min_d': 0.2,
            'update_min_a': 0.2,
        }]
    )

    return [
#        amcl_cmd,
#        map_server_cmd,
        create_bringup_cmd,
        sensors_cmd,
#        slam_toolbox_cmd,
#        gzserver_cmd,
#       gzclient_cmd,
        robot_state_publisher_cmd,
#        spawn_turtlebot_cmd,
        cartographer_cmd,
#        nav2_cmd,
#        demo_cmd,
]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
