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

#include "whisper_ros/whisper_node.hpp"
#include "common.h"
#include "whisper.h"

using namespace whisper_ros;
using std::placeholders::_1;

WhisperNode::WhisperNode() : rclcpp::Node("whisper_node") {

  std::string model;
  auto wparams = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);

  this->declare_parameters<int32_t>("", {
                                            {"n_threads", 8},
                                            {"n_max_text_ctx", 16384},
                                            {"offset_ms", 0},
                                            {"duration_ms", 0},
                                            {"max_len", 0},
                                            {"max_tokens", 32},
                                            {"audio_ctx", 0},
                                        });
  this->declare_parameters<std::string>("", {
                                                {"model", ""},
                                                {"language", "en"},
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
                                      });
  this->declare_parameters<bool>("", {
                                         {"translate", false},
                                         {"no_context", true},
                                         {"single_segment", true},
                                         {"print_special", false},
                                         {"print_progress", false},
                                         {"print_realtime", false},
                                         {"print_timestamps", false},
                                         {"token_timestamps", false},
                                         {"split_on_word", false},
                                         {"speed_up", false},
                                         {"detect_language", false},
                                         {"suppress_blank", true},
                                         {"suppress_non_speech_tokens", false},
                                     });

  this->get_parameter("model", model);

  this->get_parameter("n_threads", wparams.n_threads);
  this->get_parameter("n_max_text_ctx", wparams.n_max_text_ctx);
  this->get_parameter("offset_ms", wparams.offset_ms);
  this->get_parameter("duration_ms", wparams.duration_ms);

  this->get_parameter("translate", wparams.translate);
  this->get_parameter("no_context", wparams.no_context);
  this->get_parameter("single_segment", wparams.single_segment);
  this->get_parameter("print_special", wparams.print_special);
  this->get_parameter("print_progress", wparams.print_progress);
  this->get_parameter("print_realtime", wparams.print_realtime);
  this->get_parameter("print_timestamps", wparams.print_timestamps);

  this->get_parameter("token_timestamps", wparams.token_timestamps);
  this->get_parameter("thold_pt", wparams.thold_pt);
  this->get_parameter("thold_ptsum", wparams.thold_ptsum);
  this->get_parameter("max_len", wparams.max_len);
  this->get_parameter("split_on_word", wparams.split_on_word);
  this->get_parameter("max_tokens", wparams.max_tokens);

  this->get_parameter("speed_up", wparams.speed_up);
  this->get_parameter("audio_ctx", wparams.audio_ctx);

  this->get_parameter("language", this->language);
  wparams.language = this->language.c_str();
  this->get_parameter("detect_language", wparams.detect_language);

  this->get_parameter("suppress_blank", wparams.suppress_blank);
  this->get_parameter("suppress_non_speech_tokens",
                      wparams.suppress_non_speech_tokens);

  this->get_parameter("temperature", wparams.temperature);
  this->get_parameter("max_initial_ts", wparams.max_initial_ts);
  this->get_parameter("length_penalty", wparams.length_penalty);

  this->get_parameter("temperature_inc", wparams.temperature_inc);
  this->get_parameter("entropy_thold", wparams.entropy_thold);
  this->get_parameter("logprob_thold", wparams.logprob_thold);
  this->get_parameter("no_speech_thold", wparams.no_speech_thold);

  this->whisper = std::make_shared<Whisper>(model, wparams);
  this->publisher_ =
      this->create_publisher<std_msgs::msg::String>("whisper", 10);
  this->subscription_ =
      this->create_subscription<std_msgs::msg::Float32MultiArray>(
          "/silero_vad", 10, std::bind(&WhisperNode::vad_callback, this, _1));

  RCLCPP_INFO(this->get_logger(), "Whisper node started");
}

void WhisperNode::vad_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg) {

  RCLCPP_INFO(this->get_logger(), "Transcribing");
  std::string text_heard = trim(this->whisper->transcribe(msg->data));
  RCLCPP_INFO(this->get_logger(), "Text heard: %s", text_heard.c_str());

  std_msgs::msg::String result_msg;
  result_msg.data = text_heard;
  this->publisher_->publish(result_msg);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WhisperNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}