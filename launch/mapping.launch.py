import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from nav2_common.launch import HasNodeParams, RewrittenYaml



def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('floor_plan_experiments')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')


    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')

    # Variables
    lifecycle_nodes = ['map_saver']

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='debug',
        description='log level')


    rviz_cmd = IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'rviz_config': rviz_config_file}.items())

    robile_gazebo_cmd = IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [bringup_dir, '/launch/start_gazebo_with_robile.launch.py']))


    start_navigation_cmd = IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [bringup_dir, '/launch/nav2.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items())
        

    start_slam_toolbox_cmd = IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            slam_launch_file),
            launch_arguments={'use_sim_time': use_sim_time}.items())

    commands = [
        declare_namespace_cmd,
        declare_use_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        declare_use_composition_cmd,
        declare_use_respawn_cmd,
        declare_rviz_config_file_cmd,
        declare_use_rviz_cmd,
        rviz_cmd,
        #start_navigation_cmd,
        robile_gazebo_cmd,
        #start_slam_toolbox_cmd
    ]


    return launch.LaunchDescription(commands)



if __name__ == '__main__':
    generate_launch_description()
