
#ifndef WHISPER_NODE_HPP
#define WHISPER_NODE_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "whisper_ros/whisper.hpp"

namespace whisper_ros {

class WhisperNode : public rclcpp::Node {

public:
  WhisperNode();

private:
  std::string language;

  std::shared_ptr<Whisper> whisper;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      subscription_;

  void vad_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
};

} // namespace whisper_ros

#endif