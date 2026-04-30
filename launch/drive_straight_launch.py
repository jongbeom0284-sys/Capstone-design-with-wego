from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('limo_ros2_application')
    parameters_dir = os.path.join(pkg_dir, 'params')
    drive_params = os.path.join(parameters_dir, 'drive_straight.yaml')

    return LaunchDescription([
        Node(
            package='limo_ros2_application',
            executable='drive_straight',
            name='drive_straight',
            parameters=[drive_params],
        ),
    ])

