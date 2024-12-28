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

#ifndef WHISPER_ROS__WHISPER_SERVER_NODE_HPP
#define WHISPER_ROS__WHISPER_SERVER_NODE_HPP

#include <condition_variable>
#include <memory>
#include <mutex>

#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "whisper_msgs/action/stt.hpp"
#include "whisper_msgs/msg/transcription.hpp"
#include "whisper_ros/whisper_base_node.hpp"

namespace whisper_ros {

/**
 * @class WhisperServerNode
 * @brief A node providing speech-to-text (STT) functionality using Whisper and
 * Silero VAD.
 */
class WhisperServerNode : public WhisperBaseNode {

  using STT = whisper_msgs::action::STT;
  using GoalHandleSTT = rclcpp_action::ServerGoalHandle<STT>;

public:
  /**
   * @brief Constructor for the WhisperServerNode class.
   */
  WhisperServerNode();

  /**
   * @brief Activates ROS interfaces including subscriptions, action servers,
   * and clients.
   */
  void activate_ros_interfaces();

  /**
   * @brief Deactivates ROS interfaces by releasing subscriptions, action
   * servers, and clients.
   */
  void deactivate_ros_interfaces();

protected:
  /**
   * @brief Enables or disables Silero VAD.
   * @param enable If true, enables Silero VAD; otherwise, disables it.
   */
  void enable_silero(bool enable);

private:
  /// The transcription message containing the converted text.
  whisper_msgs::msg::Transcription transcription_msg;

  /// Mutex for synchronizing access to the transcription message.
  std::mutex transcription_mutex;

  /// Condition variable for waiting on transcription updates.
  std::condition_variable transcription_cond;

  /// Handle for the active STT goal.
  std::shared_ptr<GoalHandleSTT> goal_handle_;

  /// Subscription to receive VAD data.
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      subscription_;

  /// Client to enable or disable Silero VAD.
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_silero_client_;

  /// Action server for handling STT requests.
  rclcpp_action::Server<STT>::SharedPtr action_server_;

  /**
   * @brief Callback for processing VAD data.
   * @param msg The incoming VAD data message.
   */
  void vad_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  /**
   * @brief Executes the STT goal.
   * @param goal_handle The goal handle representing the active STT request.
   */
  void execute(const std::shared_ptr<GoalHandleSTT> goal_handle);

  /**
   * @brief Handles an incoming goal request.
   * @param uuid The unique identifier of the goal.
   * @param goal The goal details.
   * @return Response indicating whether the goal is accepted or rejected.
   */
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const STT::Goal> goal);

  /**
   * @brief Handles a cancellation request for an active goal.
   * @param goal_handle The handle of the goal to be canceled.
   * @return Response indicating whether the cancellation is accepted.
   */
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleSTT> goal_handle);

  /**
   * @brief Handles acceptance of a goal and starts execution.
   * @param goal_handle The handle of the accepted goal.
   */
  void handle_accepted(const std::shared_ptr<GoalHandleSTT> goal_handle);
};

} // namespace whisper_ros

#endif
