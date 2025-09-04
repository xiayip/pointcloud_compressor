from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_compressor',
            executable='pointcloud_decompressor_node',
            name='pointcloud_decompressor',
            output='screen',
            parameters=[{
                'input_topic': 'compressed_pointcloud',
                'output_topic': 'decompressed_pointcloud'
            }],
        )
    ])