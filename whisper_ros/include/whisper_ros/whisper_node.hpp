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

#ifndef WHISPER_ROS__WHISPER_NODE_HPP
#define WHISPER_ROS__WHISPER_NODE_HPP

#include <memory>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/empty.hpp>

#include "whisper_msgs/msg/transcription.hpp"
#include "whisper_msgs/srv/set_grammar.hpp"
#include "whisper_msgs/srv/set_init_prompt.hpp"
#include "whisper_ros/whisper_base_node.hpp"

namespace whisper_ros {

/**
 * @class WhisperNode
 * @brief A ROS 2 node that extends WhisperBaseNode to handle
 * transcription-related functionality.
 */
class WhisperNode : public WhisperBaseNode {

public:
  /**
   * @brief Constructs a WhisperNode instance.
   */
  WhisperNode();

  /**
   * @brief Activates ROS 2 interfaces such as publishers, subscriptions, and
   * services.
   */
  void activate_ros_interfaces();

  /**
   * @brief Deactivates ROS 2 interfaces by resetting and clearing publishers,
   * subscriptions, and services.
   */
  void deactivate_ros_interfaces();

private:
  /// Publisher for transcription messages.
  rclcpp::Publisher<whisper_msgs::msg::Transcription>::SharedPtr publisher_;

  /// Subscription for voice activity detection (VAD) data.
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      subscription_;

  /// Service for setting grammar configuration.
  rclcpp::Service<whisper_msgs::srv::SetGrammar>::SharedPtr
      set_grammar_service_;

  /// Service for resetting grammar configuration.
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_grammar_service_;

  /// Service for setting the initial transcription prompt.
  rclcpp::Service<whisper_msgs::srv::SetInitPrompt>::SharedPtr
      set_init_prompt_service_;

  /// Service for resetting the initial transcription prompt.
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_init_prompt_service_;

  /**
   * @brief Callback for processing voice activity detection (VAD) messages.
   *
   * @param msg A shared pointer to the received Float32MultiArray message.
   */
  void vad_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  /**
   * @brief Callback for the SetGrammar service.
   *
   * @param request Shared pointer to the request containing grammar
   * configuration.
   * @param response Shared pointer to the response indicating success or
   * failure.
   */
  void set_grammar_service_callback(
      const std::shared_ptr<whisper_msgs::srv::SetGrammar::Request> request,
      std::shared_ptr<whisper_msgs::srv::SetGrammar::Response> response);

  /**
   * @brief Callback for the ResetGrammar service.
   *
   * @param request Shared pointer to the Empty request (unused).
   * @param response Shared pointer to the Empty response (unused).
   */
  void reset_grammar_service_callback(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);

  /**
   * @brief Callback for the SetInitPrompt service.
   *
   * @param request Shared pointer to the request containing the initial prompt
   * text.
   * @param response Shared pointer to the response indicating success.
   */
  void set_init_prompt_service_callback(
      const std::shared_ptr<whisper_msgs::srv::SetInitPrompt::Request> request,
      std::shared_ptr<whisper_msgs::srv::SetInitPrompt::Response> response);

  /**
   * @brief Callback for the ResetInitPrompt service.
   *
   * @param request Shared pointer to the Empty request (unused).
   * @param response Shared pointer to the Empty response (unused).
   */
  void reset_init_prompt_service_callback(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);
};

} // namespace whisper_ros

#endif
