"""Launch file for PointCloudCompressorNode as a composable node."""

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description for composable point cloud compressor."""
    return LaunchDescription([
        # Create composable node container
        ComposableNodeContainer(
            name='pointcloud_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='pointcloud_compressor',
                    plugin='PointCloudCompressorNode',
                    name='pointcloud_compressor_node',
                    parameters=[{
                        'input_topic': 'odin1/cloud_slam', 
                        'output_topic': 'compressed_pointcloud',
                        'compression_level': 6,  # default compression level
                        'quantization_bits': 16  # default quantization bits
                    }],
                ),
            ],
            output='screen',
        ),
    ])
