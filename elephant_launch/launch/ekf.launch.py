import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    # Set the path to different files and folders.
    # pkg_share = FindPackageShare(package='elephant_launch').find('elephant_launch')
    robot_localization_file_path = os.path.join(
        get_package_share_directory('elephant_launch'),
        'config',
        'ekf.yaml') 

  # Start robot localization using an Extended Kalman filter
    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path])

    odom = Node(package='elephant_odom',executable='wheel_odometry')
    imu = Node(package='elephant_imu',executable='imu')
    can = Node(package='elephant_can',executable='can')

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(start_robot_localization_cmd)

    ld.add_action(odom)
    ld.add_action(imu)
    ld.add_action(can)

    return ld