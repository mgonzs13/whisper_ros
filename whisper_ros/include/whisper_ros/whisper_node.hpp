
#ifndef WHISPER_NODE_HPP
#define WHISPER_NODE_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "common-sdl.h"
#include "whisper_ros/whisper.hpp"

namespace whisper_ros {

class WhisperNode : public rclcpp::Node {

public:
  WhisperNode();
  void work();

private:
  int32_t voice_ms;
  float vad_thold;
  float freq_thold;
  bool print_energy;

  std::shared_ptr<Whisper> whisper;
  std::shared_ptr<audio_async> audio;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

} // namespace whisper_ros

#endif