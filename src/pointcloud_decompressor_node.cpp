#include "pointcloud_compressor/pointcloud_codec.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

class PointCloudDecompressorNode : public rclcpp::Node
{
public:
  explicit PointCloudDecompressorNode(const rclcpp::NodeOptions & options)
      : Node("pointcloud_decompressor_node", options)
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

  std::cout<<"input topic: "<<input_topic<<std::endl;
  std::cout<<"output topic: "<<output_topic<<std::endl;
    // Create publisher for decompressed point clouds
    decompressed_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);

    // Create subscription for encoded data
    encoded_sub_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
        input_topic, 10,
        std::bind(&PointCloudDecompressorNode::decodeCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "PointCloud Decompressor Node started");
  }

private:
  void decodeCallback(const std_msgs::msg::ByteMultiArray::SharedPtr msg)
  {
    try {
      // Create message for decoded data
      sensor_msgs::msg::PointCloud2 decompressed_msg;
      
      // Log encoded data info
      RCLCPP_DEBUG(this->get_logger(), "Received encoded data with %zu bytes", msg->data.size());
      
      // Start timing
      auto start = std::chrono::high_resolution_clock::now();
      
      // Decode the point cloud
      if (!codec_->decode(*msg, decompressed_msg)) {
        throw std::runtime_error("Decoding failed");
      }
      
      // Calculate decompression stats
      auto end = std::chrono::high_resolution_clock::now();
      double decompression_time = std::chrono::duration<double>(end - start).count();
      
      // Publish decompressed data
      decompressed_pub_->publish(decompressed_msg);
      
      // Log performance metrics
      RCLCPP_INFO(this->get_logger(), 
          "Decompressed %zu bytes to %zu bytes in %.3f seconds",
          msg->data.size(), decompressed_msg.data.size(), decompression_time);
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error decompressing point cloud: %s", e.what());
    }
  }

  std::unique_ptr<pointcloud_compressor::PointCloudCodec> codec_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr decompressed_pub_;
  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr encoded_sub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudDecompressorNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// Register the component with the ROS2 component system
RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudDecompressorNode)