from ament_index_python.packages import get_package_share_path

from launch_ros.actions import Node, PushRosNamespace
from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
)

from launch.substitutions import LaunchConfiguration

def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()

    # Namespace argument
    arg = DeclareLaunchArgument('ns')
    launch_description.add_action(arg)

    # Get path to config and add. 
    # Declaring a parameter to load in launch file.
    package_path = get_package_share_path('dory_nav2')
    param_file_path = f'{package_path}/dory_waypt_params.yaml'
    param_file_arg = DeclareLaunchArgument('dory_config_file',
                                           default_value=param_file_path)
    launch_description.add_action(param_file_arg)

    # Declare node to use. 
    node = Node(executable='gen_force',
                package='dory_nav2',
                parameters=[
                    LaunchConfiguration('dory_config_file'),
                ])

    # For grouping things into namespaces (ROS2) version.
    group = GroupAction([
        PushRosNamespace(LaunchConfiguration('ns')),
        node,
    ])
    launch_description.add_action(group)


    return launch_description