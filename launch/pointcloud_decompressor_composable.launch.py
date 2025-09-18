"""Launch file for PointCloudDecompressorNode as a composable node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description for composable point cloud decompressor."""
    return LaunchDescription([
        # Create composable node container
        ComposableNodeContainer(
            name=LaunchConfiguration('container_name'),
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='pointcloud_compressor',
                    plugin='PointCloudDecompressorNode',
                    name='pointcloud_decompressor_node',
                    parameters=[{
                        'input_topic': 'odin1/cloud_slam', 
                        'output_topic': 'compressed_pointcloud',
                    }],
                ),
            ],
            output='screen',
        ),
    ])
