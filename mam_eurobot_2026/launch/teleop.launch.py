from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess


def generate_launch_description():
    pkg_share = get_package_share_directory('mam_eurobot_2026')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('headless', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'arena.launch.py')
            ),
            launch_arguments={'rviz': LaunchConfiguration('rviz'), 'headless': LaunchConfiguration('headless')}.items()
        ),
        # Try to run the teleop script directly with python3. Prefer installed package share path;
        # fallback to the workspace src path if running from source.
        ExecuteProcess(
            cmd=[
                'python3',
                os.path.join(os.path.dirname(pkg_share), 'src', 'mam_eurobot_2026', 'teleop', 'teleop_keyboard.py')
            ],
            output='screen'
        )
    ])
