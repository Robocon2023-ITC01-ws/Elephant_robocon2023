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

    serial_port_arg = DeclareLaunchArgument(
        "serial_port", default_value = TextSubstitution(text = "/dev/ttyUSB0")
    )
    imu_link_arg = DeclareLaunchArgument(
        "frame_id", default_value = TextSubstitution(text = "imu_link")
    )
    IMU_arg = DeclareLaunchArgument(
        "operation_mode", default_value = TextSubstitution(text = "IMU")
    )
    INTERNAL_arg = DeclareLaunchArgument(
        "oscillator", default_value = TextSubstitution(text = "INTERNAL")
    )
    reset_orientation_arg = DeclareLaunchArgument(
        "reset_orientation", default_value = TextSubstitution(text = "true")
    )
    frequency_arg = DeclareLaunchArgument(
        "frequency", default_value = TextSubstitution(text = "50")
    )
    ros_imu_node = Node(
        package = 'elephant_imu',
        name = 'imu_node',
        executable = 'imu',
        parameters = [{
            "serial_port": LaunchConfiguration('serial_port'),
            "frame_id": LaunchConfiguration('frame_id'),
            "operation_mode": LaunchConfiguration('operation_mode'),
            "oscillator": LaunchConfiguration('oscillator'),
            "reset_orientation": LaunchConfiguration('reset_orientation'),
            "frequency": LaunchConfiguration('frequency'),
        }]
    )

    return LaunchDescription([
        serial_port_arg,
        imu_link_arg,
        IMU_arg,
        INTERNAL_arg,
        reset_orientation_arg,
        frequency_arg,
        ros_imu_node,
        Node(package='elephant_localization', executable='localization'),
        #Node(package='rabbit_can',executable='can'),
        Node(package='joy', executable='joy_node'),
        #Node(package='rabbit_can', executable='joy'),
    ])


   
