# PointCloud Compressor

A ROS2 package for compressing and decompressing point cloud data using Google's Draco compression library. This package provides efficient point cloud compression with support for both geometric (XYZ) and color (RGB) data.

## Overview

The PointCloud Compressor package consists of two main components:
- **Compressor Node**: Subscribes to PointCloud2 messages and publishes compressed data
- **Decompressor Node**: Subscribes to compressed data and publishes reconstructed PointCloud2 messages

The package uses Google's Draco compression library through a custom `dracopy.hpp` wrapper to achieve high compression ratios while maintaining point cloud quality.

## Features

- **High Compression Ratio**: Achieves significant size reduction for point cloud data
- **RGB Support**: Handles both XYZ-only and XYZRGB point clouds
- **Configurable Compression**: Adjustable compression levels and quantization bits
- **ROS2 Native**: Seamless integration with ROS2 ecosystem
- **Real-time Performance**: Optimized for real-time point cloud processing

## Dependencies

- ROS2 Humble (or compatible)
- Google Draco compression library
- sensor_msgs
- std_msgs
- rclcpp

## Installation

1. Clone this package into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository_url>
```

2. Install dependencies:
```bash
# Install Draco library
sudo apt-get install libdraco-dev
# or build from source if needed
```

3. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select pointcloud_compressor
```

4. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### Compressor Node

Compresses incoming PointCloud2 messages and publishes compressed data.

```bash
ros2 run pointcloud_compressor pointcloud_compressor_node
```

#### Parameters

- `input_topic` (string, default: "input_pointcloud"): Input PointCloud2 topic
- `output_topic` (string, default: "encoded_pointcloud"): Output compressed data topic
- `compression_level` (int, default: 10): Compression level (0-10, higher = better compression)
- `quantization_bits` (int, default: 16): Quantization bits (lower = higher compression)

#### Topics

**Subscribed:**
- `input_pointcloud` (sensor_msgs/PointCloud2): Input point cloud data

**Published:**
- `encoded_pointcloud` (std_msgs/ByteMultiArray): Compressed point cloud data

### Decompressor Node

Decompresses compressed data and publishes reconstructed PointCloud2 messages.

```bash
ros2 run pointcloud_compressor pointcloud_decompressor_node
```

#### Parameters

- `input_topic` (string, default: "encoded_pointcloud"): Input compressed data topic
- `output_topic` (string, default: "decompressed_pointcloud"): Output PointCloud2 topic
- `compression_level` (int, default: 10): Compression level (must match compressor)
- `quantization_bits` (int, default: 16): Quantization bits (must match compressor)

#### Topics

**Subscribed:**
- `encoded_pointcloud` (std_msgs/ByteMultiArray): Compressed point cloud data

**Published:**
- `decompressed_pointcloud` (sensor_msgs/PointCloud2): Reconstructed point cloud data

### Launch Files

You can create launch files to run both nodes together:

```python
# launch/pointcloud_compression.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_compressor',
            executable='pointcloud_compressor_node',
            name='pointcloud_compressor',
            parameters=[{
                'input_topic': '/camera/points',
                'output_topic': '/compressed_points',
                'compression_level': 7,
                'quantization_bits': 14
            }]
        ),
        Node(
            package='pointcloud_compressor',
            executable='pointcloud_decompressor_node',
            name='pointcloud_decompressor',
            parameters=[{
                'input_topic': '/compressed_points',
                'output_topic': '/decompressed_points',
                'compression_level': 7,
                'quantization_bits': 14
            }]
        )
    ])
```

## Configuration

### Compression Settings

- **Compression Level (0-10)**:
  - 0: Fastest compression, larger file size
  - 10: Best compression, slower processing
  - Recommended: 7-8 for balanced performance

- **Quantization Bits (8-16)**:
  - 8: Highest compression, lower precision
  - 16: Lower compression, higher precision
  - Recommended: 14-16 for most applications

### Performance Tuning

For real-time applications, consider:
- Lower compression levels (5-7)
- Higher quantization bits (14-16)
- Monitor CPU usage and adjust accordingly

## Data Formats

### Input PointCloud2 Format
The compressor accepts standard ROS2 PointCloud2 messages with:
- **Required fields**: x, y, z (FLOAT32)
- **Optional fields**: rgb (UINT32)

### Compressed Data Format
Compressed data is published as `std_msgs/ByteMultiArray` containing:
- Draco-encoded geometry data
- Draco-encoded color data (if present)
- Metadata for reconstruction

## Performance Metrics

Typical compression ratios (dependent on data characteristics):
- **XYZ-only point clouds**: 3:1 to 8:1 compression ratio
- **XYZRGB point clouds**: 4:1 to 12:1 compression ratio
- **Processing latency**: 5-50ms (depending on point count and settings)

## Examples

### Basic Usage

```bash
# Terminal 1: Start compressor
ros2 run pointcloud_compressor pointcloud_compressor_node \
  --ros-args -p input_topic:=/velodyne_points -p compression_level:=8

# Terminal 2: Start decompressor  
ros2 run pointcloud_compressor pointcloud_decompressor_node \
  --ros-args -p output_topic:=/compressed_velodyne_points

# Terminal 3: Monitor compression ratio
ros2 topic echo /encoded_pointcloud
```

### Integration with Other Nodes

```bash
# Compress point clouds from a sensor
ros2 run pointcloud_compressor pointcloud_compressor_node \
  --ros-args -p input_topic:=/camera/depth/points \
  --ros-args -p output_topic:=/camera/compressed_points

# Visualize decompressed data in RViz
ros2 run rviz2 rviz2
# Add PointCloud2 display for /decompressed_pointcloud topic
```

## API Reference

### PointCloudCodec Class

Main compression/decompression functionality:

```cpp
class PointCloudCodec {
public:
    PointCloudCodec(rclcpp::Node& node, int compression_level, int quantization_bits);
    
    bool encode(const sensor_msgs::msg::PointCloud2& cloud, 
                std_msgs::msg::ByteMultiArray& encoded_data);
    
    bool decode(const std_msgs::msg::ByteMultiArray& data, 
                sensor_msgs::msg::PointCloud2& decoded_cloud);
};
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

[License information to be added]

## Changelog

### Version 0.0.0
- Initial implementation
- Support for XYZ and RGB point cloud compression
- Basic compression/decompression nodes
- Draco integration through dracopy.hpp wrapper
