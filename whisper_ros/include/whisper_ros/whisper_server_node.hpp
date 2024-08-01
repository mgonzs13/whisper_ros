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

#ifndef WHISPER_SERVER_NODE_HPP
#define WHISPER_SERVER_NODE_HPP

#include <condition_variable>
#include <memory>
#include <mutex>

#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "whisper_msgs/action/stt.hpp"
#include "whisper_ros/whisper_base_node.hpp"

namespace whisper_ros {

class WhisperServerNode : public WhisperBaseNode {

  using STT = whisper_msgs::action::STT;
  using GoalHandleSTT = rclcpp_action::ServerGoalHandle<STT>;

public:
  WhisperServerNode();

protected:
  void enable_silero(bool enable);

private:
  std::string text;
  std::mutex text_mutex;
  std::condition_variable text_cond;

  std::shared_ptr<GoalHandleSTT> goal_handle_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      subscription_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_silero_client_;
  rclcpp_action::Server<STT>::SharedPtr action_server_;

  void vad_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void execute(const std::shared_ptr<GoalHandleSTT> goal_handle);
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const STT::Goal> goal);
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleSTT> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleSTT> goal_handle);
};

} // namespace whisper_ros

#endif