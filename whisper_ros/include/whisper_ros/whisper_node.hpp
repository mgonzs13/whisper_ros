
#ifndef WHISPER_NODE_HPP
#define WHISPER_NODE_HPP

#include <memory>

#include <audio_common_msgs/msg/audio_data.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "whisper_ros/whisper.hpp"

namespace whisper_ros {

class WhisperNode : public rclcpp::Node {

public:
  WhisperNode();

private:
  void audio_callback(const audio_common_msgs::msg::AudioData::SharedPtr msg);
  bool msg_to_wav(std::vector<uint8_t> data, std::vector<float> *pcmf32);

  std::shared_ptr<Whisper> whisper;

  rclcpp::Subscription<audio_common_msgs::msg::AudioData>::SharedPtr
      subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

} // namespace whisper_ros

#endif