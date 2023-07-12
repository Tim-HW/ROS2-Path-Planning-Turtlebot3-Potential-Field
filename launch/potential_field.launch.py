from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([

        Node(package='potential_field',
        executable='potential_field_node',#same as Cmakelist
        output='screen'),
        #Node(
        #    package='turtlebot3_gazebo',
        #    executable='turtlebot3_dqn_stage3.launch.py',
        #),

    ])