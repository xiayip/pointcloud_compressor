from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_compressor',
            executable='pointcloud_compressor_node',
            name='pointcloud_compressor',
            output='screen',
            parameters=[{
                'input_topic': 'odin1/cloud_slam', 
                'output_topic': 'compressed_pointcloud',
                'compression_level': 6,  # 默认压缩等级
                'quantization_bits': 16  # 默认量化等级
            }],
        )
    ])