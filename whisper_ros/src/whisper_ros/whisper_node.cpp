// MIT License
//
// Copyright (c) 2023 Miguel Ángel González Santamarta
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "whisper_ros/whisper_node.hpp"
#include "whisper.h"

using namespace whisper_ros;
using std::placeholders::_1;
using std::placeholders::_2;

WhisperNode::WhisperNode() : WhisperBaseNode() {
  RCLCPP_INFO(this->get_logger(), "Whisper node started");
}

void WhisperNode::activate_ros_interfaces() {
  // services
  this->set_grammar_service_ =
      this->create_service<whisper_msgs::srv::SetGrammar>(
          "set_grammar",
          std::bind(&WhisperNode::set_grammar_service_callback, this, _1, _2));
  this->reset_grammar_service_ = this->create_service<std_srvs::srv::Empty>(
      "reset_grammar",
      std::bind(&WhisperNode::reset_grammar_service_callback, this, _1, _2));
  this->set_init_prompt_service_ =
      this->create_service<whisper_msgs::srv::SetInitPrompt>(
          "set_init_prompt",
          std::bind(&WhisperNode::set_init_prompt_service_callback, this, _1,
                    _2));
  this->reset_init_prompt_service_ = this->create_service<std_srvs::srv::Empty>(
      "reset_init_prompt",
      std::bind(&WhisperNode::reset_init_prompt_service_callback, this, _1,
                _2));

  // pubs, subs
  this->publisher_ = this->create_publisher<whisper_msgs::msg::Transcription>(
      "transcription", 10);
  this->subscription_ =
      this->create_subscription<std_msgs::msg::Float32MultiArray>(
          "vad", 10, std::bind(&WhisperNode::vad_callback, this, _1));
}

void WhisperNode::deactivate_ros_interfaces() {

  this->set_grammar_service_.reset();
  this->set_grammar_service_ = nullptr;

  this->reset_grammar_service_.reset();
  this->reset_grammar_service_ = nullptr;

  this->set_init_prompt_service_.reset();
  this->set_init_prompt_service_ = nullptr;

  this->reset_init_prompt_service_.reset();
  this->reset_init_prompt_service_ = nullptr;

  this->publisher_.reset();
  this->publisher_ = nullptr;

  this->subscription_.reset();
  this->subscription_ = nullptr;
}

void WhisperNode::vad_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg) {

  auto transcription_msg = this->transcribe(msg->data);
  this->publisher_->publish(transcription_msg);
}

void WhisperNode::set_grammar_service_callback(
    const std::shared_ptr<whisper_msgs::srv::SetGrammar::Request> request,
    std::shared_ptr<whisper_msgs::srv::SetGrammar::Response> response) {

  response->success = this->whisper->set_grammar(
      request->grammar_config.grammar,        // grammar text
      request->grammar_config.start_rule,     // grammar start rule
      request->grammar_config.grammar_penalty // grammar penalty
  );
}

void WhisperNode::reset_grammar_service_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {

  (void)request;
  (void)response;

  this->whisper->reset_grammar();
}

void WhisperNode::set_init_prompt_service_callback(
    const std::shared_ptr<whisper_msgs::srv::SetInitPrompt::Request> request,
    std::shared_ptr<whisper_msgs::srv::SetInitPrompt::Response> response) {

  this->whisper->set_init_prompt(request->prompt);
  response->success = true;
}

void WhisperNode::reset_init_prompt_service_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {

  (void)request;
  (void)response;

  this->whisper->reset_init_prompt();
}