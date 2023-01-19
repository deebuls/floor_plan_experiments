import os
import sys

import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_rviz = LaunchConfiguration('use_rviz')
    pose = {'x': LaunchConfiguration('x_pose', default='2.00'),
            'y': LaunchConfiguration('y_pose', default='-1.0'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')
    
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start RVIZ')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start RVIZ')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start RVIZ')
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("robile_description"),
                    "gazebo",
                    "gazebo_robile.xacro"
                ]
            ),
            " ",
            "platform_config:=4_wheel_config",
            " ",
            "movable_joints:=False",
        ]
    )

    robot_state_pub_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': ParameterValue(robot_description_content, value_type=str)
            }],
    )

    spawn_robot_gazebo_cmd = Node(package=    'gazebo_ros',
                                  executable= 'spawn_entity.py',
                                  arguments=  ['-topic', 'robot_description',
                                               '-entity', 'robile',
                                               '-x', pose['x'], '-y', pose['y']
                                              ],
                                  output=     'screen')


    static_transform_cmd = Node(package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            arguments=["0","0","0","0","0","0","base_footprint","base_link"]
    )

    nodes = [
        robot_state_pub_cmd,
        spawn_robot_gazebo_cmd,
        static_transform_cmd
    ]
    return launch.LaunchDescription(nodes)


if __name__ == '__main__':
    generate_launch_description()
