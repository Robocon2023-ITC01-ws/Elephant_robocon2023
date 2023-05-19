import os
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    # args that can be set from the command line or a default will be used

    return LaunchDescription([
        Node(package='elephant_can',executable='can'),
        Node(package='elephant_localization', executable='localization'),
        Node(package='elephant_can',executable='teleop'),
        #Node(package='elephant_control',executable='test_controller'),
        Node(package='elephant_shooter',executable='shooter'),
        Node(package='joy', executable='joy_node'),
        #Node(package='elephant_odom',executable='wheel_odometry'),
        #Node(package='elephant_publish_pos', executable='test_pos_pub'),
    ])


   
