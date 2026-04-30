import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def generate_launch_description():
    # LIMO base (motor driver) bringup
    port_name_arg = DeclareLaunchArgument(
        'port_name',
        default_value='ttyTHS1',
        description='Serial port name under /dev (e.g. ttyTHS1, ttyUSB0, ttylimo)',
    )

    drive_mode_arg = DeclareLaunchArgument(
        'drive_mode',
        default_value='ackermann',
        description='Drive mode: ackermann | diff | mecanum',
    )

    force_ackermann_mode_arg = DeclareLaunchArgument(
        'force_ackermann_mode',
        default_value='true',
        description='If true, ackermann driver sends commands even if status motion_mode is invalid',
    )

    limo_base_share = get_package_share_directory('limo_base')
    base_launch_ack = os.path.join(limo_base_share, 'launch', 'limo_base_ackerman.launch.py')
    base_launch_diff = os.path.join(limo_base_share, 'launch', 'limo_base.launch.py')
    base_launch_mec = os.path.join(limo_base_share, 'launch', 'limo_base_mecanum.launch.py')

    def _make_limo_base(context, *args, **kwargs):
        mode = LaunchConfiguration('drive_mode').perform(context).strip().lower()
        if mode in ('ackermann', 'ackerman'):
            launch_path = base_launch_ack
        elif mode in ('mecanum', 'mcnamu', 'omni'):
            launch_path = base_launch_mec
        else:
            launch_path = base_launch_diff

        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_path),
                launch_arguments={
                    'port_name': LaunchConfiguration('port_name'),
                    'force_ackermann_mode': LaunchConfiguration('force_ackermann_mode'),
                }.items(),
            ),
            Node(
                package='limo_ros2_application',
                executable='drive_straight',
                name='drive_straight',
                parameters=[
                    drive_params,
                    {
                        'publish_ackermann': mode in ('ackermann', 'ackerman'),
                        'ack_cmd_topic': '/limo/ack_cmd',
                    }
                ],
            ),
        ]

    app_share = get_package_share_directory('limo_ros2_application')
    params_dir = os.path.join(app_share, 'params')
    drive_params = os.path.join(params_dir, 'drive_straight.yaml')

    return LaunchDescription([
        port_name_arg,
        drive_mode_arg,
        force_ackermann_mode_arg,
        OpaqueFunction(function=_make_limo_base),
    ])

