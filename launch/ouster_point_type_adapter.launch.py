from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='pointcloud_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='ouster_point_type_adapter',
                    plugin='ouster_point_type_adapter::OusterPointTypeAdapter',
                    name='ouster_point_type_adapter',
                    remappings=[('input', '/ouster/points'),
                                ('output', '/ouster/aw_points')],
                )
            ],
            output='screen',
        )
    ])
