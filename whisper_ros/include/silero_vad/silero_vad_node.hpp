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

#ifndef SILERO_VAD__SILERO_VAD_NODE_HPP
#define SILERO_VAD__SILERO_VAD_NODE_HPP

#include <atomic>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "audio_common_msgs/msg/audio_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "silero_vad/vad_iterator.hpp"

namespace silero_vad {

/// @class SileroVadNode
/// @brief A ROS 2 lifecycle node for performing voice activity detection.
class SileroVadNode : public rclcpp_lifecycle::LifecycleNode {

public:
  /// @brief Constructs a new SileroVadNode object.
  SileroVadNode();

  /// @brief Callback for configuring the lifecycle node.
  /// @param state The current state of the node.
  /// @return Success or failure of configuration.
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &state);

  /// @brief Callback for activating the lifecycle node.
  /// @param state The current state of the node.
  /// @return Success or failure of activation.
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &state);

  /// @brief Callback for deactivating the lifecycle node.
  /// @param state The current state of the node.
  /// @return Success or failure of deactivation.
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &state);

  /// @brief Callback for cleaning up the lifecycle node.
  /// @param state The current state of the node.
  /// @return Success or failure of cleanup.
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &state);

  /// @brief Callback for shutting down the lifecycle node.
  /// @param state The current state of the node.
  /// @return Success or failure of shutdown.
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &state);

protected:
  /// Indicates if VAD is enabled.
  std::atomic<bool> enabled;
  /// Indicates if VAD is in listening mode.
  std::atomic<bool> listening;
  /// Indicates if audio data should be published.
  std::atomic<bool> publish;
  /// Buffer for storing audio data.
  std::vector<float> data;
  /// Pointer to the VAD iterator.
  std::unique_ptr<VadIterator> vad_iterator;

private:
  /// Buffer for storing previous audio data.
  std::vector<float> prev_data;
  /// Path to the VAD model.
  std::string model_path_;
  /// Sampling rate of the audio data.
  int sample_rate_;
  /// Frame size in milliseconds.
  int frame_size_ms_;
  /// Threshold for VAD decision-making.
  float threshold_;
  /// Minimum silence duration in milliseconds.
  int min_silence_ms_;
  /// Padding duration for detected speech in milliseconds.
  int speech_pad_ms_;
  /// Indicates if the CUDA provider should be used.
  bool use_cuda_;

  /// Publisher for VAD output.
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  /// Subscription for audio input.
  rclcpp::Subscription<audio_common_msgs::msg::AudioStamped>::SharedPtr
      subscription_;
  /// Service for enabling/disabling VAD.
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;

  /// @brief Callback for handling incoming audio data.
  /// @param msg The audio message containing the audio data.
  void
  audio_callback(const audio_common_msgs::msg::AudioStamped::SharedPtr msg);

  /// @brief Callback for enabling/disabling the VAD.
  /// @param request The service request containing the desired enable state.
  /// @param response The service response indicating success or failure.
  void enable_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                 std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  /// @brief Converts audio data to a float vector normalized to [-1.0, 1.0].
  /// @tparam T The input audio data type.
  /// @param input The input audio data.
  /// @return A vector of normalized float audio data.
  template <typename T>
  std::vector<float> convert_to_float(const std::vector<T> &input) {
    static_assert(std::is_integral<T>::value,
                  "Input type must be an integral type.");

    std::vector<float> output;
    output.reserve(input.size());

    if constexpr (std::is_same<T, uint8_t>::value) {
      // uint8_t data normalized to [0.0, 1.0]
      for (T value : input) {
        output.push_back(static_cast<float>(value) / 255.0f);
      }
    } else if constexpr (std::is_same<T, int8_t>::value) {
      // int8_t data normalized to [-1.0, 1.0]
      constexpr float scale = 1.0f / 127.0f; // Max positive value of int8_t
      for (T value : input) {
        output.push_back(static_cast<float>(value) * scale);
      }
    } else if constexpr (std::is_same<T, int16_t>::value) {
      // int16_t data normalized to [-1.0, 1.0]
      constexpr float scale = 1.0f / 32767.0f; // Max positive value of int16_t
      for (T value : input) {
        output.push_back(static_cast<float>(value) * scale);
      }
    } else if constexpr (std::is_same<T, int32_t>::value) {
      // int32_t data normalized to [-1.0, 1.0]
      constexpr float scale =
          1.0f / 2147483647.0f; // Max positive value of int32_t
      for (T value : input) {
        output.push_back(static_cast<float>(value) * scale);
      }
    } else {
      throw std::invalid_argument(
          "Unsupported data type for audio conversion.");
    }

    return output;
  }
};

} // namespace silero_vad

#endif
