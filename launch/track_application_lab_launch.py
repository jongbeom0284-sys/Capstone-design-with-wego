from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('limo_ros2_application')
    parameters_dir = os.path.join(pkg_dir, 'params')
    lane_params = os.path.join(parameters_dir, 'lab_adaptive_lane.yaml')
    limo_control_parameters = os.path.join(parameters_dir, 'limo_control.yaml')

    return LaunchDescription([
        Node(
            package='limo_ros2_application',
            executable='lab_adaptive_lane',
            name='lab_adaptive_lane',
            parameters=[lane_params],
        ),
        Node(
            package='limo_ros2_application',
            executable='limo_e_stop',
            name='limo_e_stop',
        ),
        Node(
            package='limo_ros2_application',
            executable='limo_control',
            name='limo_control',
            parameters=[limo_control_parameters],
        ),
    ])

