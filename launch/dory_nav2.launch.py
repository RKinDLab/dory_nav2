import os
from datetime import datetime
import launch
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node



def generate_launch_description():


    #####################
    ## BlueROV2 Launch
    #####################
    # Get the directory where the main dory2 launch file is located
    sub_package_share_directory = get_package_share_directory('ros2_control_blue_reach_5')
    # Create a path to the dory launch file
    sub_launch_path = f'{sub_package_share_directory}/launch/robot_system_multi_interface.launch.py'

    # Include the dory launch file
    sub_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sub_launch_path),
        launch_arguments={'robot_controller':'forward_effort_controller',
                          'use_mock_hardware':'true'
                          }.items()
    )

    #####################
    ## BAG HANDLE
    #####################
    # Rosbag flag argument
    record_bag_arg = DeclareLaunchArgument(
            'record_bag',
            default_value='false',
            description='Flag to enable or disable rosbag recording'
        )
    # path to rosbag directory
    workspace_directory = os.getcwd()
    timestamp = datetime.now().strftime('%Y_%m_%d-%H_%M_%S')
    relative_path = f'bags/dory_response_{timestamp}'
    bags_filepath = os.path.join(workspace_directory, relative_path)

    # record_rosbag if flag True
    record_bag = LaunchConfiguration('record_bag')
    state_thrust_bag = ExecuteProcess(
                                      condition=IfCondition(record_bag),
                                      cmd=['ros2', 'bag', 'record',
                                           '-o', bags_filepath,
                                           '--storage', 'sqlite3',
                                           '/state_thrust_vector'],
                                      output='screen'
                                      )
    #####################
    ## COMMAND VELOCITY HANDLE
    #####################
    # Get the directory where the command_vel pub launch file is located
    sub_package_share_directory = get_package_share_directory('dory_nav2')
    # Create a path to the dory launch file
    sub_launch_path = f'{sub_package_share_directory}/dynamic_reconfig.launch.py'

    # command velocity node
    # gen_cmd_vel_node = Node(
    #                         package='dory_nav2',
    #                         executable='gen_cmd_vel',
    #                         name='command_vel_publisher',
    #                         output='screen',
    #                         parameters = [{'use_sim_time':False}]
    #                         )
    gen_cmd_vel_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sub_launch_path),
        launch_arguments={'testing':'testing',
                    }.items()
        )

    # thrust node
    gen_thrust_node = Node(
                           package='dory_nav2',
                           executable='gen_force',
                           name='thrust_publisher',
                           output='screen',
                           )

    #####################
    ## ODOM TF HANDLE
    #####################
    # Odometry TF node
    odom_pub = Node(
                           package='dory_nav2',
                           executable='odom_publisher',
                           #name='odom_publisher',
                           output='screen',
                           )   

    return LaunchDescription([
        #record_bag_arg,
        sub_launch,
        gen_cmd_vel_node,
        #gen_thrust_node,
        #state_thrust_bag
        odom_pub
    ])