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

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <portaudio.h>

#include "silero_vad/silero_vad_node.hpp"
#include "whisper_utils/model_download.hpp"

using namespace silero_vad;
using std::placeholders::_1;
using std::placeholders::_2;

SileroVadNode::SileroVadNode()
    : rclcpp_lifecycle::LifecycleNode("silero_vad_node"), listening(false),
      publish(false) {

  this->declare_parameter<bool>("enabled", true);
  this->declare_parameter<std::string>("model_repo", "");
  this->declare_parameter<std::string>("model_filename", "");
  this->declare_parameter<std::string>("model_path", "");
  this->declare_parameter<int>("sample_rate", 16000);
  this->declare_parameter<int>("frame_size_ms", 32);
  this->declare_parameter<float>("threshold", 0.5f);
  this->declare_parameter<int>("min_silence_ms", 100);
  this->declare_parameter<int>("speech_pad_ms", 30);
  this->declare_parameter<bool>("use_cuda", false);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SileroVadNode::on_configure(const rclcpp_lifecycle::State &) {

  RCLCPP_INFO(this->get_logger(), "[%s] Configuring...", this->get_name());

  // get params
  bool enabled;
  this->get_parameter("enabled", enabled);
  this->enabled.store(enabled);

  std::string model_repo;
  std::string model_filename;

  this->get_parameter("model_repo", model_repo);
  this->get_parameter("model_filename", model_filename);
  this->get_parameter("model_path", this->model_path_);
  this->get_parameter("sample_rate", this->sample_rate_);
  this->get_parameter("frame_size_ms", this->frame_size_ms_);
  this->get_parameter("threshold", this->threshold_);
  this->get_parameter("min_silence_ms", this->min_silence_ms_);
  this->get_parameter("speech_pad_ms", this->speech_pad_ms_);
  this->get_parameter("use_cuda", this->use_cuda_);

  // download model
  if (this->model_path_.empty()) {
    this->model_path_ =
        whisper_utils::download_model(model_repo, model_filename);
  }

  RCLCPP_INFO(this->get_logger(), "[%s] Configured", this->get_name());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SileroVadNode::on_activate(const rclcpp_lifecycle::State &) {

  RCLCPP_INFO(this->get_logger(), "[%s] Activating...", this->get_name());

  // create silero-vad
  this->vad_iterator = std::make_unique<VadIterator>(
      this->model_path_, this->sample_rate_, this->frame_size_ms_,
      this->threshold_, this->min_silence_ms_, this->speech_pad_ms_,
      this->use_cuda_);

  this->publisher_ =
      this->create_publisher<std_msgs::msg::Float32MultiArray>("vad", 10);
  this->subscription_ =
      this->create_subscription<audio_common_msgs::msg::AudioStamped>(
          "audio", rclcpp::SensorDataQoS(),
          std::bind(&SileroVadNode::audio_callback, this, _1));

  this->enable_srv_ = this->create_service<std_srvs::srv::SetBool>(
      "enable_vad", std::bind(&SileroVadNode::enable_cb, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "[%s] Activated", this->get_name());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SileroVadNode::on_deactivate(const rclcpp_lifecycle::State &) {

  RCLCPP_INFO(this->get_logger(), "[%s] Deactivating...", this->get_name());

  // reset silero
  this->vad_iterator->reset_states();

  this->publisher_.reset();
  this->publisher_ = nullptr;

  this->subscription_.reset();
  this->subscription_ = nullptr;

  this->enable_srv_.reset();
  this->enable_srv_ = nullptr;

  RCLCPP_INFO(this->get_logger(), "[%s] Deactivated", this->get_name());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SileroVadNode::on_cleanup(const rclcpp_lifecycle::State &) {

  RCLCPP_INFO(this->get_logger(), "[%s] Cleaning up...", this->get_name());
  RCLCPP_INFO(this->get_logger(), "[%s] Cleaned up", this->get_name());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SileroVadNode::on_shutdown(const rclcpp_lifecycle::State &) {

  RCLCPP_INFO(this->get_logger(), "[%s] Shutting down...", this->get_name());
  RCLCPP_INFO(this->get_logger(), "[%s] Shutted down", this->get_name());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

void SileroVadNode::audio_callback(
    const audio_common_msgs::msg::AudioStamped::SharedPtr msg) {

  if (!this->enabled) {
    return;
  }

  std::vector<float> data;

  try {
    switch (msg->audio.info.format) {
    case paFloat32: {
      data = msg->audio.audio_data.float32_data;
      break;
    }
    case paInt32: {
      data = this->convert_to_float<int32_t>(msg->audio.audio_data.int32_data);
      break;
    }
    case paInt16: {
      data = this->convert_to_float<int16_t>(msg->audio.audio_data.int16_data);
      break;
    }
    case paInt8: {
      data = this->convert_to_float<int8_t>(msg->audio.audio_data.int8_data);
      break;
    }
    case paUInt8: {
      data = this->convert_to_float<uint8_t>(msg->audio.audio_data.uint8_data);
      break;
    }
    default:
      RCLCPP_ERROR(this->get_logger(), "Unsupported format");
      return;
    }

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Error while processing audio data: %s",
                 e.what());
    return;
  }

  // Predict if speech starts or ends
  auto timestamp = this->vad_iterator->predict(data);

  // Check if speech starts
  if (timestamp.start != -1 && timestamp.end == -1 && !this->listening) {
    RCLCPP_INFO(this->get_logger(), "Speech starts...");
    this->listening.store(true);
    this->data.clear();

    if (this->prev_data.size()) {
      this->data.insert(this->data.end(), this->prev_data.begin(),
                        this->prev_data.end());
    }
  }

  // Add audio if listening
  if (this->listening) {
    this->data.insert(this->data.end(), data.begin(), data.end());
  }

  // Check if publish
  if (this->publish) {
    auto vad_msg = std_msgs::msg::Float32MultiArray();
    vad_msg.data.assign(this->data.begin(), this->data.end());

    if (vad_msg.data.size() / msg->audio.info.rate < 1.0) {
      int pad_size =
          msg->audio.info.chunk + msg->audio.info.rate - this->data.size();
      vad_msg.data.insert(vad_msg.data.end(), pad_size, 0.0f);
    }

    this->publisher_->publish(vad_msg);

    this->listening.store(false);
    this->publish.store(false);
    this->data.clear();
  }

  // Check if speech ends
  if (timestamp.start == -1 && timestamp.end != -1 && this->listening) {
    RCLCPP_INFO(this->get_logger(), "Speech ends...");
    this->publish.store(true);
  }

  this->prev_data = data;
}

void SileroVadNode::enable_cb(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

  response->success = true;

  if (request->data && !this->enabled) {
    response->message = "SileroVAD enabled";
    this->enabled.store(true);
    this->listening.store(false);
    this->publish.store(false);
    this->data.clear();
    this->prev_data.clear();
    this->vad_iterator->reset_states();

  } else if (request->data && this->enabled) {
    response->message = "SileroVAD already enabled";

  } else if (!request->data && this->enabled) {
    response->message = "SileroVAD disabled";
    this->enabled.store(false);
    this->listening.store(false);
    this->publish.store(false);
    this->data.clear();
    this->prev_data.clear();

  } else if (!request->data && !this->enabled) {
    response->message = "SileroVAD already disabled";
  }

  RCLCPP_INFO(this->get_logger(), response->message.c_str());
}
