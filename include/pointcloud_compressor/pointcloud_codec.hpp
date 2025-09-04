#ifndef POINTCLOUD_COMPRESSOR__POINTCLOUD_CODEC_HPP_
#define POINTCLOUD_COMPRESSOR__POINTCLOUD_CODEC_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

#include <draco/compression/expert_encode.h>
#include <draco/compression/decode.h>
#include <draco/compression/encode.h>
#include <draco/point_cloud/point_cloud.h>
#include <draco/point_cloud/point_cloud_builder.h>

namespace pointcloud_compressor
{

  class PointCloudCodec
  {
  public:
    explicit PointCloudCodec(rclcpp::Node &node,int compression_level, int quantization_bits);

    bool encode(const sensor_msgs::msg::PointCloud2 &cloud, std_msgs::msg::ByteMultiArray &encoded_data);

    bool decode(const std_msgs::msg::ByteMultiArray &data, sensor_msgs::msg::PointCloud2 &decoded_cloud);

  private:
    rclcpp::Logger logger_;

    // Compression parameters
    int compression_level_;
    int quantization_bits_;
    bool use_lossless_;

    // Draco encoder/decoder
    std::unique_ptr<draco::ExpertEncoder> encoder_;
    std::unique_ptr<draco::Decoder> decoder_;
  };

} // namespace pointcloud_compressor

#endif // POINTCLOUD_COMPRESSOR__POINTCLOUD_CODEC_HPP_