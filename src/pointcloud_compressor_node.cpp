#include "pointcloud_compressor/pointcloud_codec.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

class PointCloudCompressorNode : public rclcpp::Node
{
public:
  PointCloudCompressorNode()
      : Node("pointcloud_compressor_node")
  {
    // Declare parameters
    this->declare_parameter("input_topic", "input_pointcloud");
    this->declare_parameter("output_topic", "encoded_pointcloud");
    this->declare_parameter("compression_level", 10);
    this->declare_parameter("quantization_bits", 16);
    

    // Get parameter values
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    int compression_level = this->get_parameter("compression_level").as_int();
    int quantization_bits = this->get_parameter("quantization_bits").as_int();

    // Initialize codec
    codec_ = std::make_unique<pointcloud_compressor::PointCloudCodec>(*this,compression_level,quantization_bits);

    // Create publisher for encoded data
    encoded_pub_ = this->create_publisher<std_msgs::msg::ByteMultiArray>(output_topic, 10);

    // Create subscription for point cloud data
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, 10,
        std::bind(&PointCloudCompressorNode::pointcloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "PointCloud Compressor Node started");
  }

private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    try {
      // Create message for encoded data
      std_msgs::msg::ByteMultiArray encoded_msg;
      
      // Log point cloud info
      RCLCPP_DEBUG(this->get_logger(), "Received point cloud with %d points", msg->width * msg->height);
      
      // Start timing
      auto start = std::chrono::high_resolution_clock::now();
      
      // Encode the point cloud
      if (!codec_->encode(*msg, encoded_msg)) {
        throw std::runtime_error("Encoding failed");
      }
      
      // Calculate compression stats
      auto end = std::chrono::high_resolution_clock::now();
      double compression_time = std::chrono::duration<double>(end - start).count();
      double compression_ratio = static_cast<double>(encoded_msg.data.size()) / msg->data.size();
      
      // Publish encoded data
      encoded_pub_->publish(encoded_msg);
      
      // Log performance metrics
      RCLCPP_INFO(this->get_logger(), 
          "Compressed %zu bytes to %zu bytes (ratio: %.2f) in %.3f seconds",
          msg->data.size(), encoded_msg.data.size(), compression_ratio, compression_time);
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error compressing point cloud: %s", e.what());
    }
  }

  std::unique_ptr<pointcloud_compressor::PointCloudCodec> codec_;
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr encoded_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudCompressorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}