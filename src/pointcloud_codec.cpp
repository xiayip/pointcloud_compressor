#include "pointcloud_compressor/pointcloud_codec.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <memory>
#include <draco/compression/expert_encode.h>
#include <draco/compression/decode.h>
#include <draco/core/encoder_buffer.h>
#include <draco/core/status.h>
#include <draco/point_cloud/point_cloud.h>
#include "pointcloud_compressor/pointcloud_codec.hpp"
#include "dracopy.hpp"
namespace pointcloud_compressor
{

  PointCloudCodec::PointCloudCodec(rclcpp::Node &node,int compression_level, int quantization_bits)
      : logger_(node.get_logger()),
        compression_level_(compression_level),  // higher the level,lower the speed
        quantization_bits_(quantization_bits), // lower the bits,higer the compression rate
        use_lossless_(true)
  {
    // 创建一个空的 draco::PointCloud 对象
    draco::PointCloud point_cloud;
    encoder_ = std::make_unique<draco::ExpertEncoder>(point_cloud);
    decoder_ = std::make_unique<draco::Decoder>();
    RCLCPP_INFO(logger_, "PointCloudCodec initialized with default compression settings");
  }

  bool PointCloudCodec::encode(const sensor_msgs::msg::PointCloud2 &cloud, std_msgs::msg::ByteMultiArray &encoded_data)
  {
    // 检查点云数据是否有效
    if (cloud.data.empty() || cloud.width * cloud.height == 0)
    {
      RCLCPP_ERROR(logger_, "PointCloud2 message is empty");
      return false;
    }

    // 获取 x, y, z 字段的偏移量
    int x_offset = -1, y_offset = -1, z_offset = -1, rgb_offset = -1;
    for (const auto &field : cloud.fields)
    {
      if (field.name == "x")
        x_offset = field.offset;
      if (field.name == "y")
        y_offset = field.offset;
      if (field.name == "z")
        z_offset = field.offset;
      if (field.name == "rgb")
        rgb_offset = field.offset;
    }

    // 检查是否找到 x, y, z 字段
    if (x_offset == -1 || y_offset == -1 || z_offset == -1)
    {
      RCLCPP_ERROR(logger_, "PointCloud2 message does not contain x, y, z fields");
      return false;
    }

    // 提取点云数据
    std::vector<float> points;
    std::vector<uint8_t> colors;
    for (size_t i = 0; i < cloud.width * cloud.height; i++)
    {
      const uint8_t *point_ptr = &cloud.data[0] + i * cloud.point_step;

      // 提取 x, y, z 值
      float x = *reinterpret_cast<const float *>(point_ptr + x_offset);
      float y = *reinterpret_cast<const float *>(point_ptr + y_offset);
      float z = *reinterpret_cast<const float *>(point_ptr + z_offset);

      // 将点数据添加到 points 向量
      points.push_back(x);
      points.push_back(y);
      points.push_back(z);

      // 提取 RGB 值
      if (rgb_offset != -1)
      {
        uint32_t rgb = *reinterpret_cast<const uint32_t *>(point_ptr + rgb_offset);
        colors.push_back((rgb >> 16) & 0xFF);  // R
        colors.push_back((rgb >> 8) & 0xFF);   // G
        colors.push_back(rgb & 0xFF);          // B
      }
    }

    // 调用 Draco 编码函数
    uint8_t colors_channel = rgb_offset != -1 ? 3 : 0;
    DracoFunctions::EncodedObject obj = DracoFunctions::encode_point_cloud(
        points, quantization_bits_, compression_level_, 0.0, nullptr, false, false, 0, colors, colors_channel);

    // 检查编码是否成功
    if (obj.encode_status != DracoFunctions::successful_encoding)
    {
      RCLCPP_ERROR(logger_, "Failed to encode point cloud");
      return false;
    }

    // 将编码后的数据复制到 ByteMultiArray
    encoded_data.data.assign(obj.buffer.begin(), obj.buffer.end());
    RCLCPP_DEBUG(logger_, "Successfully encoded point cloud (%zu bytes)", encoded_data.data.size());
    return true;
  }

  bool PointCloudCodec::decode(const std_msgs::msg::ByteMultiArray &data, sensor_msgs::msg::PointCloud2 &decoded_cloud)
  {
    DracoFunctions::MeshObject obj = DracoFunctions::decode_buffer((const char *)data.data.data(), data.data.size());
    if (obj.decode_status != DracoFunctions::successful)
    {
      RCLCPP_ERROR(logger_, "Failed to decode point cloud");
      return false;
    }
    // Check if RGB data is available
    bool has_rgb = !obj.colors.empty() && obj.colors.size() == obj.points.size();
    RCLCPP_INFO(logger_, "has_rgb=%s, point size=%zu color size=%zu", 
                has_rgb ? "true" : "false", obj.points.size(), obj.colors.size());

    sensor_msgs::msg::PointCloud2 ros_cloud;
    ros_cloud.header.frame_id = "map";
    ros_cloud.height = 1;
    ros_cloud.width = obj.points.size() / 3;
    ros_cloud.is_dense = true;
    
    sensor_msgs::PointCloud2Modifier modifier(ros_cloud);
    if (has_rgb) {
      modifier.setPointCloud2Fields(
          4,
          "x", 1, sensor_msgs::msg::PointField::FLOAT32,
          "y", 1, sensor_msgs::msg::PointField::FLOAT32,
          "z", 1, sensor_msgs::msg::PointField::FLOAT32,
          "rgb", 1, sensor_msgs::msg::PointField::FLOAT32
      );
    } else {
      modifier.setPointCloud2Fields(
          3,
          "x", 1, sensor_msgs::msg::PointField::FLOAT32,
          "y", 1, sensor_msgs::msg::PointField::FLOAT32,
          "z", 1, sensor_msgs::msg::PointField::FLOAT32
      );
    }
    modifier.resize(ros_cloud.width * ros_cloud.height);
    
    sensor_msgs::PointCloud2Iterator<float> iter_x(ros_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(ros_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(ros_cloud, "z");
    
    // RGB iterator (only initialize if RGB data is available)
    std::unique_ptr<sensor_msgs::PointCloud2Iterator<float>> iter_rgb;
    if (has_rgb) {
      iter_rgb = std::make_unique<sensor_msgs::PointCloud2Iterator<float>>(ros_cloud, "rgb");
    }
    for (size_t i = 0; i < obj.points.size(); i += 3)
    {
      *iter_x = obj.points[i];
      *iter_y = obj.points[i + 1];
      *iter_z = obj.points[i + 2];
      ++iter_x;
      ++iter_y;
      ++iter_z;
      
      // Process RGB data if available
      if (has_rgb && i < obj.colors.size() && iter_rgb) {
        uint32_t rgb = (static_cast<uint32_t>(obj.colors[i]) << 16) |     // R
                       (static_cast<uint32_t>(obj.colors[i + 1]) << 8) |   // G
                       static_cast<uint32_t>(obj.colors[i + 2]);           // B
        float rgb_float;
        std::memcpy(&rgb_float, &rgb, sizeof(float));
        **iter_rgb = rgb_float;
        ++(*iter_rgb);
      }

      // std::cout << "x: " << obj.points[i] << " y: " << obj.points[i + 1] << " z: " << obj.points[i + 2] << std::endl;
    }

    // Set up the decoded_cloud with proper header and fields
    decoded_cloud = ros_cloud;
    decoded_cloud.header.stamp = rclcpp::Clock().now();
    decoded_cloud.is_dense = false;

    RCLCPP_DEBUG(logger_, "Successfully decoded point cloud with%s RGB data", has_rgb ? "" : "out");
    return true;
  }

} // namespace pointcloud_compressor