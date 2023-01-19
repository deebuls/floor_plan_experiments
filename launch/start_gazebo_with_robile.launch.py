import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                [get_package_share_directory(
                    'floor_plan_experiments'), '/launch/hospital.launch.py']
            ),
            launch_arguments={
                'gui': 'true'
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                [get_package_share_directory(
                    'floor_plan_experiments'), '/launch/robile_gazebo.launch.py']
            )
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
