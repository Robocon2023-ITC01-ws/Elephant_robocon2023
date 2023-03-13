import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Set the path to different files and folders.
    pkg_share = FindPackageShare(package='elephant_launch').find('elephant_launch')
    robot_localization_file_path = os.path.join(pkg_share, 'config/ekf.yaml') 


  # Launch configuration variables specific to simulation
    # use_sim_time = LaunchConfiguration('use_sim_time')

  # Start robot localization using an Extended Kalman filter
    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path])


    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(start_robot_localization_cmd)

    return ld