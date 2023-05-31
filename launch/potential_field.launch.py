from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([Node(package='potential_field',
                                   executable='potential_field_node',#same as Cmakelist
                                   output='screen')]) 