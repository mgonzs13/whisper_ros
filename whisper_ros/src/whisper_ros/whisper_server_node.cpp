// MIT License
//
// Copyright (c) 2024 Miguel Ángel González Santamarta
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

#include <memory>

#include "whisper.h"
#include "whisper_ros/whisper_server_node.hpp"

using namespace whisper_ros;
using std::placeholders::_1;
using std::placeholders::_2;

WhisperServerNode::WhisperServerNode() : WhisperBaseNode() {
  RCLCPP_INFO(this->get_logger(), "WhisperServer node started");
}

void WhisperServerNode::activate_ros_interfaces() {
  this->enable_silero_client_ =
      this->create_client<std_srvs::srv::SetBool>("enable_vad");

  this->subscription_ =
      this->create_subscription<std_msgs::msg::Float32MultiArray>(
          "vad", 10, std::bind(&WhisperServerNode::vad_callback, this, _1));

  this->goal_handle_ = nullptr;
  this->action_server_ = rclcpp_action::create_server<STT>(
      this, "listen", std::bind(&WhisperServerNode::handle_goal, this, _1, _2),
      std::bind(&WhisperServerNode::handle_cancel, this, _1),
      std::bind(&WhisperServerNode::handle_accepted, this, _1));
}

void WhisperServerNode::deactivate_ros_interfaces() {

  this->enable_silero_client_.reset();
  this->enable_silero_client_ = nullptr;

  this->subscription_.reset();
  this->subscription_ = nullptr;

  this->action_server_.reset();
  this->action_server_ = nullptr;
}

void WhisperServerNode::enable_silero(bool enable) {

  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = enable;

  this->enable_silero_client_->wait_for_service();
  this->enable_silero_client_->async_send_request(req);
}

void WhisperServerNode::vad_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg) {

  this->enable_silero(false);

  this->transcription_msg = this->transcribe(msg->data);

  this->transcription_cond.notify_all();
}

rclcpp_action::GoalResponse
WhisperServerNode::handle_goal(const rclcpp_action::GoalUUID &uuid,
                               std::shared_ptr<const STT::Goal> goal) {
  (void)uuid;
  (void)goal;

  if (this->goal_handle_ != nullptr && this->goal_handle_->is_active()) {
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WhisperServerNode::handle_cancel(
    const std::shared_ptr<GoalHandleSTT> goal_handle) {
  (void)goal_handle;

  RCLCPP_INFO(this->get_logger(), "Received request to cancel Whisper node");
  this->enable_silero(false);
  this->transcription_cond.notify_all();

  return rclcpp_action::CancelResponse::ACCEPT;
}

void WhisperServerNode::handle_accepted(
    const std::shared_ptr<GoalHandleSTT> goal_handle) {
  this->goal_handle_ = goal_handle;
  std::thread{std::bind(&WhisperServerNode::execute, this, _1), goal_handle}
      .detach();
}

void WhisperServerNode::execute(
    const std::shared_ptr<GoalHandleSTT> goal_handle) {

  // get goal data
  this->goal_handle_ = goal_handle;
  auto goal = goal_handle->get_goal();

  if (!goal->grammar_config.grammar.empty()) {
    this->whisper->set_grammar(goal->grammar_config.grammar,
                               goal->grammar_config.start_rule,
                               goal->grammar_config.grammar_penalty);
  }
  this->whisper->set_init_prompt(goal->prompt);

  auto result = std::make_shared<STT::Result>();
  this->transcription_msg.text.clear();

  this->enable_silero(true);

  // wait for text
  while (this->transcription_msg.text.empty() && !goal_handle->is_canceling()) {
    std::unique_lock<std::mutex> lock(this->transcription_mutex);
    this->transcription_cond.wait(lock);
  }

  // reset
  this->whisper->reset_grammar();
  this->whisper->reset_init_prompt();
  this->enable_silero(false);

  if (goal_handle->is_canceling()) {
    goal_handle->canceled(result);

  } else {
    result->transcription = this->transcription_msg;
    goal_handle->succeed(result);
  }
}