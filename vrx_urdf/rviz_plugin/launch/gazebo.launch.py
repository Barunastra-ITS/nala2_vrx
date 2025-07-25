import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    rviz_config_path = os.path.join(get_package_share_directory('rviz_plugin'), 'rviz', 'gazebo.rviz')
    rviz_plugin_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        rviz_plugin_node,
    ])
