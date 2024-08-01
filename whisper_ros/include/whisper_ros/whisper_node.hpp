// MIT License

// Copyright (c) 2023  Miguel Ángel González Santamarta

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef WHISPER_NODE_HPP
#define WHISPER_NODE_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>

#include "whisper_msgs/srv/set_grammar.hpp"
#include "whisper_msgs/srv/set_init_prompt.hpp"
#include "whisper_ros/whisper.hpp"

namespace whisper_ros {

class WhisperNode : public rclcpp::Node {

public:
  WhisperNode();

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      subscription_;

  rclcpp::Service<whisper_msgs::srv::SetGrammar>::SharedPtr
      set_grammar_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_grammar_service_;
  rclcpp::Service<whisper_msgs::srv::SetInitPrompt>::SharedPtr
      set_init_prompt_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_init_prompt_service_;

  void vad_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void set_grammar_service_callback(
      const std::shared_ptr<whisper_msgs::srv::SetGrammar::Request> request,
      std::shared_ptr<whisper_msgs::srv::SetGrammar::Response> response);
  void reset_grammar_service_callback(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);
  void set_init_prompt_service_callback(
      const std::shared_ptr<whisper_msgs::srv::SetInitPrompt::Request> request,
      std::shared_ptr<whisper_msgs::srv::SetInitPrompt::Response> response);
  void reset_init_prompt_service_callback(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);

protected:
  std::string language;
  std::shared_ptr<Whisper> whisper;
};

} // namespace whisper_ros

#endif