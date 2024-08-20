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

#include <regex>

#include "whisper.h"
#include "whisper_ros/whisper_base_node.hpp"

using namespace whisper_ros;
using std::placeholders::_1;
using std::placeholders::_2;

WhisperBaseNode::WhisperBaseNode()
    : rclcpp_lifecycle::LifecycleNode("whisper_node") {

  this->declare_parameters<int32_t>("", {
                                            {"n_threads", 8},
                                            {"n_max_text_ctx", 16384},
                                            {"offset_ms", 0},
                                            {"duration_ms", 0},
                                            {"max_len", 0},
                                            {"max_tokens", 0},
                                            {"audio_ctx", 0},
                                            {"greedy_best_of", 5},
                                            {"beam_search_beam_size", 5},
                                            {"n_processors", 1},
                                            {"gpu_device", 0},
                                        });
  this->declare_parameters<std::string>(
      "", {
              {"sampling_strategy", "beam_search"},
              {"model", ""},
              {"language", "en"},
              {"openvino_encode_device", "CPU"},
          });
  this->declare_parameters<float>("", {
                                          {"thold_pt", 0.01f},
                                          {"thold_ptsum", 0.01f},
                                          {"temperature", 0.00f},
                                          {"max_initial_ts", 1.00f},
                                          {"length_penalty", -1.00f},
                                          {"temperature_inc", 0.40f},
                                          {"entropy_thold", 2.40f},
                                          {"logprob_thold", -1.00f},
                                          {"no_speech_thold", 0.60f},
                                          {"beam_search_patience", -1.00f},
                                      });
  this->declare_parameters<bool>("", {
                                         {"translate", false},
                                         {"no_context", true},
                                         {"no_timestamps", false},
                                         {"single_segment", true},
                                         {"print_special", false},
                                         {"print_progress", false},
                                         {"print_realtime", false},
                                         {"print_timestamps", false},
                                         {"token_timestamps", false},
                                         {"split_on_word", false},
                                         {"speed_up", false},
                                         {"tinydiarize", false},
                                         {"detect_language", false},
                                         {"suppress_blank", true},
                                         {"suppress_non_speech_tokens", false},
                                         {"use_gpu", true},
                                     });
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WhisperBaseNode::on_configure(const rclcpp_lifecycle::State &) {

  RCLCPP_INFO(get_logger(), "[%s] Configuring...", this->get_name());

  // get sampling method and create default params
  std::string sampling_strategy;
  this->get_parameter("sampling_strategy", sampling_strategy);

  if (sampling_strategy == "greedy") {
    this->wparams = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);
  } else if (sampling_strategy == "beam_search") {
    this->wparams = whisper_full_default_params(WHISPER_SAMPLING_BEAM_SEARCH);
  }

  // get params
  this->get_parameter("model", this->model);
  this->get_parameter("openvino_encode_device", this->openvino_encode_device);

  this->get_parameter("n_threads", this->wparams.n_threads);
  this->get_parameter("n_max_text_ctx", this->wparams.n_max_text_ctx);
  this->get_parameter("offset_ms", this->wparams.offset_ms);
  this->get_parameter("duration_ms", this->wparams.duration_ms);

  this->get_parameter("translate", this->wparams.translate);
  this->get_parameter("no_context", this->wparams.no_context);
  this->get_parameter("no_timestamps", this->wparams.no_timestamps);
  this->get_parameter("single_segment", this->wparams.single_segment);
  this->get_parameter("print_special", this->wparams.print_special);
  this->get_parameter("print_progress", this->wparams.print_progress);
  this->get_parameter("print_realtime", this->wparams.print_realtime);
  this->get_parameter("print_timestamps", this->wparams.print_timestamps);

  this->get_parameter("token_timestamps", this->wparams.token_timestamps);
  this->get_parameter("thold_pt", this->wparams.thold_pt);
  this->get_parameter("thold_ptsum", this->wparams.thold_ptsum);
  this->get_parameter("max_len", this->wparams.max_len);
  this->get_parameter("split_on_word", this->wparams.split_on_word);
  this->get_parameter("max_tokens", this->wparams.max_tokens);

  this->get_parameter("audio_ctx", this->wparams.audio_ctx);
  this->get_parameter("tinydiarize", this->wparams.tdrz_enable);

  this->get_parameter("language", this->language);
  this->wparams.language = this->language.c_str();
  this->get_parameter("detect_language", this->wparams.detect_language);

  this->get_parameter("suppress_blank", this->wparams.suppress_blank);
  this->get_parameter("suppress_non_speech_tokens",
                      this->wparams.suppress_non_speech_tokens);

  this->get_parameter("temperature", this->wparams.temperature);
  this->get_parameter("max_initial_ts", this->wparams.max_initial_ts);
  this->get_parameter("length_penalty", this->wparams.length_penalty);

  this->get_parameter("temperature_inc", this->wparams.temperature_inc);
  this->get_parameter("entropy_thold", this->wparams.entropy_thold);
  this->get_parameter("logprob_thold", this->wparams.logprob_thold);
  this->get_parameter("no_speech_thold", this->wparams.no_speech_thold);

  this->get_parameter("greedy_best_of", this->wparams.greedy.best_of);
  this->get_parameter("beam_search_beam_size",
                      this->wparams.beam_search.beam_size);
  this->get_parameter("beam_search_patience",
                      this->wparams.beam_search.patience);

  this->get_parameter("n_processors", this->n_processors);
  this->get_parameter("use_gpu", this->cparams.use_gpu);
  this->get_parameter("gpu_device", this->cparams.gpu_device);

  // check threads number
  if (this->wparams.n_threads < 0) {
    this->wparams.n_threads = std::thread::hardware_concurrency();
  }

  RCLCPP_INFO(get_logger(), "[%s] Configured", this->get_name());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WhisperBaseNode::on_activate(const rclcpp_lifecycle::State &) {

  RCLCPP_INFO(get_logger(), "[%s] Activating...", this->get_name());

  // create whisper
  this->whisper = std::make_shared<Whisper>(
      this->model, this->openvino_encode_device, this->n_processors,
      this->cparams, this->wparams);

  this->activate_ros_interfaces();

  RCLCPP_INFO(get_logger(), "[%s] Activated", this->get_name());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WhisperBaseNode::on_deactivate(const rclcpp_lifecycle::State &) {

  RCLCPP_INFO(get_logger(), "[%s] Deactivating...", this->get_name());

  this->whisper.reset();
  this->whisper = nullptr;

  this->deactivate_ros_interfaces();

  RCLCPP_INFO(get_logger(), "[%s] Deactivated", this->get_name());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WhisperBaseNode::on_cleanup(const rclcpp_lifecycle::State &) {

  RCLCPP_INFO(get_logger(), "[%s] Cleaning up...", this->get_name());
  RCLCPP_INFO(get_logger(), "[%s] Cleaned up", this->get_name());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WhisperBaseNode::on_shutdown(const rclcpp_lifecycle::State &) {

  RCLCPP_INFO(get_logger(), "[%s] Shutting down...", this->get_name());
  RCLCPP_INFO(get_logger(), "[%s] Shutted down", this->get_name());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

whisper_msgs::msg::Transcription
WhisperBaseNode::transcribe(const std::vector<float> &audio) {

  auto start_time = this->get_clock()->now();
  RCLCPP_INFO(this->get_logger(), "Transcribing");
  transcription_output result = this->whisper->transcribe(audio);
  std::string text = this->whisper->trim(result.text);
  auto end_time = this->get_clock()->now();

  RCLCPP_INFO(this->get_logger(), "Text heard: %s", text.c_str());
  whisper_msgs::msg::Transcription msg;
  msg.text = text;
  msg.audio_time = audio.size() / WHISPER_SAMPLE_RATE;
  msg.transcription_time = (end_time - start_time).seconds();

  return msg;
}
