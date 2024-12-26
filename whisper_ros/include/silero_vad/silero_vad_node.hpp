// MIT License

// Copyright (c) 2024  Miguel Ángel González Santamarta

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

#ifndef SILERO_VAD_NODE_HPP
#define SILERO_VAD_NODE_HPP

#include <atomic>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "audio_common_msgs/msg/audio_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "silero_vad/vad_iterator.hpp"

namespace silero_vad {

class SileroVadNode : public rclcpp_lifecycle::LifecycleNode {

public:
  SileroVadNode();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &);

protected:
  bool enabled;
  std::atomic<bool> listening;
  std::vector<float> data;
  std::unique_ptr<VadIterator> vad_iterator;

private:
  std::string model_path_;
  int sample_rate_;
  int frame_size_ms_;
  float threshold_;
  int min_silence_ms_;
  int speech_pad_ms_;
  int min_speech_ms_;
  float max_speech_s_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<audio_common_msgs::msg::AudioStamped>::SharedPtr
      subscription_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;

  void
  audio_callback(const audio_common_msgs::msg::AudioStamped::SharedPtr msg);

  void enable_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                 std::shared_ptr<std_srvs::srv::SetBool::Response> response);

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