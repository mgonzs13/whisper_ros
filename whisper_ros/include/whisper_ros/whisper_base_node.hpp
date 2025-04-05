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

#ifndef WHISPER_ROS__WHISPER_BASE_NODE_HPP
#define WHISPER_ROS__WHISPER_BASE_NODE_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "whisper_msgs/msg/transcription.hpp"
#include "whisper_ros/whisper.hpp"

namespace whisper_ros {

/**
 * @brief Alias for the CallbackReturn type from the LifecycleNodeInterface.
 *
 * This alias is used to simplify the code and improve readability.
 */
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @class WhisperBaseNode
 * @brief A ROS 2 Lifecycle Node for interfacing with the Whisper speech-to-text
 * system.
 */
class WhisperBaseNode : public rclcpp_lifecycle::LifecycleNode {

public:
  /**
   * @brief Constructs a new WhisperBaseNode instance.
   */
  WhisperBaseNode();

  /**
   * @brief Configures the node during its lifecycle.
   *
   * @param state The current state of the lifecycle node.
   * @return CallbackReturn indicating success or failure.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &state);

  /**
   * @brief Activates the node during its lifecycle.
   *
   * @param state The current state of the lifecycle node.
   * @return CallbackReturn indicating success or failure.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &state);

  /**
   * @brief Deactivates the node during its lifecycle.
   *
   * @param state The current state of the lifecycle node.
   * @return CallbackReturn indicating success or failure.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &state);

  /**
   * @brief Cleans up resources during its lifecycle.
   *
   * @param state The current state of the lifecycle node.
   * @return CallbackReturn indicating success or failure.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &state);

  /**
   * @brief Shuts down the node during its lifecycle.
   *
   * @param state The current state of the lifecycle node.
   * @return CallbackReturn indicating success or failure.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &state);

  /**
   * @brief Activates ROS 2 interfaces specific to this node. To be implemented
   * in derived classes.
   */
  virtual void activate_ros_interfaces(){};

  /**
   * @brief Deactivates ROS 2 interfaces specific to this node. To be
   * implemented in derived classes.
   */
  virtual void deactivate_ros_interfaces(){};

protected:
  /// @brief The language for transcription (e.g., "en").
  std::string language;

  /// @brief Shared pointer to the Whisper speech-to-text processor.
  std::shared_ptr<Whisper> whisper;

  /**
   * @brief Transcribes a given audio input to text.
   *
   * @param audio A vector of floating-point audio samples.
   * @return A Transcription message containing the text and metadata.
   */
  whisper_msgs::msg::Transcription transcribe(const std::vector<float> &audio);

private:
  /// @brief The Whisper model to use.
  std::string model;

  /// @brief Device identifier for OpenVINO encoding (e.g., "CPU").
  std::string openvino_encode_device;

  /// @brief Number of processors to use.
  int n_processors;

  /// @brief Whisper context parameters.
  struct whisper_context_params cparams = whisper_context_default_params();

  /// @brief Whisper full transcription parameters.
  struct whisper_full_params wparams;
};

} // namespace whisper_ros

#endif // WHISPER_ROS__WHISPER_BASE_NODE_HPP
