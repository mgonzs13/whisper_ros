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

#include "whisper.h"

#include "whisper_ros/whisper_base_node.hpp"
#include "whisper_utils/model_download.hpp"

using namespace whisper_ros;
using std::placeholders::_1;
using std::placeholders::_2;

WhisperBaseNode::WhisperBaseNode()
    : rclcpp_lifecycle::LifecycleNode("whisper_node") {

  // Model parameters
  this->declare_parameters<std::string>("model",
                                        {
                                            {"repo", ""},
                                            {"filename", ""},
                                            {"path", ""},
                                            {"openvino_encode_device", "CPU"},
                                        });

  // Sampling parameters
  this->declare_parameters<std::string>("sampling",
                                        {
                                            {"strategy", "beam_search"},
                                        });
  this->declare_parameters<int32_t>("sampling",
                                    {
                                        {"greedy_best_of", 5},
                                        {"beam_search_beam_size", 5},
                                    });
  this->declare_parameters<float>("sampling",
                                  {
                                      {"beam_search_patience", -1.00f},
                                  });

  // Transcription parameters
  this->declare_parameters<int32_t>("transcription",
                                    {
                                        {"n_threads", 4},
                                        {"n_max_text_ctx", 16384},
                                        {"n_processors", 1},
                                        {"offset_ms", 0},
                                        {"duration_ms", 0},
                                        {"audio_ctx", 0},
                                    });
  this->declare_parameters<std::string>("transcription",
                                        {
                                            {"language", "en"},
                                            {"initial_prompt", ""},
                                            {"suppress_regex", ""},
                                        });
  this->declare_parameters<bool>("transcription",
                                 {
                                     {"translate", false},
                                     {"no_context", true},
                                     {"no_timestamps", false},
                                     {"single_segment", false},
                                     {"detect_language", false},
                                     {"suppress_blank", true},
                                     {"suppress_nst", false},
                                     {"carry_initial_prompt", false},
                                 });

  // Token timestamps parameters
  this->declare_parameters<bool>("token_timestamps",
                                 {
                                     {"enabled", false},
                                     {"split_on_word", false},
                                 });
  this->declare_parameters<float>("token_timestamps",
                                  {
                                      {"thold_pt", 0.01f},
                                      {"thold_ptsum", 0.01f},
                                  });
  this->declare_parameters<int32_t>("token_timestamps", {
                                                            {"max_len", 0},
                                                            {"max_tokens", 0},
                                                        });

  // Decoding / fallback parameters
  this->declare_parameters<float>("decoding", {
                                                  {"temperature", 0.00f},
                                                  {"max_initial_ts", 1.00f},
                                                  {"length_penalty", -1.00f},
                                                  {"temperature_inc", 0.20f},
                                                  {"entropy_thold", 2.40f},
                                                  {"logprob_thold", -1.00f},
                                                  {"no_speech_thold", 0.60f},
                                              });

  // Speaker diarization parameters
  this->declare_parameters<bool>("diarization", {
                                                    {"tdrz_enable", false},
                                                });

  // GPU / backend parameters
  this->declare_parameters<bool>("gpu", {
                                            {"enabled", true},
                                            {"flash_attn", true},
                                        });
  this->declare_parameters<int32_t>("gpu", {
                                               {"device", 0},
                                           });

  // DTW token timestamps parameters
  this->declare_parameters<bool>("dtw", {
                                            {"token_timestamps", false},
                                        });
  this->declare_parameters<int32_t>("dtw", {
                                               {"n_top", -1},
                                           });
  this->declare_parameters<std::string>("dtw", {
                                                   {"aheads", "none"},
                                               });
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WhisperBaseNode::on_configure(const rclcpp_lifecycle::State &) {

  RCLCPP_INFO(this->get_logger(), "[%s] Configuring...", this->get_name());

  // get sampling method and create default params
  std::string dtw_aheads;
  std::string sampling_strategy;
  this->get_parameter("sampling.strategy", sampling_strategy);

  if (sampling_strategy == "greedy") {
    this->wparams = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);
  } else if (sampling_strategy == "beam_search") {
    this->wparams = whisper_full_default_params(WHISPER_SAMPLING_BEAM_SEARCH);
  }

  // model params
  std::string model_repo;
  std::string model_filename;

  this->get_parameter("model.repo", model_repo);
  this->get_parameter("model.filename", model_filename);
  this->get_parameter("model.path", this->model);
  this->get_parameter("model.openvino_encode_device",
                      this->openvino_encode_device);

  // transcription params
  this->get_parameter("transcription.n_threads", this->wparams.n_threads);
  this->get_parameter("transcription.n_max_text_ctx",
                      this->wparams.n_max_text_ctx);
  this->get_parameter("transcription.offset_ms", this->wparams.offset_ms);
  this->get_parameter("transcription.duration_ms", this->wparams.duration_ms);

  this->get_parameter("transcription.translate", this->wparams.translate);
  this->get_parameter("transcription.no_context", this->wparams.no_context);
  this->get_parameter("transcription.no_timestamps",
                      this->wparams.no_timestamps);
  this->get_parameter("transcription.single_segment",
                      this->wparams.single_segment);

  this->get_parameter("transcription.audio_ctx", this->wparams.audio_ctx);
  this->get_parameter("transcription.suppress_regex", this->suppress_regex);

  this->get_parameter("transcription.language", this->language);
  this->wparams.language = this->language.c_str();
  this->get_parameter("transcription.detect_language",
                      this->wparams.detect_language);

  this->get_parameter("transcription.suppress_blank",
                      this->wparams.suppress_blank);
  this->get_parameter("transcription.suppress_nst", this->wparams.suppress_nst);

  this->get_parameter("transcription.initial_prompt", this->initial_prompt);
  this->get_parameter("transcription.carry_initial_prompt",
                      this->wparams.carry_initial_prompt);

  // token timestamps params
  this->get_parameter("token_timestamps.enabled",
                      this->wparams.token_timestamps);
  this->get_parameter("token_timestamps.thold_pt", this->wparams.thold_pt);
  this->get_parameter("token_timestamps.thold_ptsum",
                      this->wparams.thold_ptsum);
  this->get_parameter("token_timestamps.max_len", this->wparams.max_len);
  this->get_parameter("token_timestamps.split_on_word",
                      this->wparams.split_on_word);
  this->get_parameter("token_timestamps.max_tokens", this->wparams.max_tokens);

  // decoding / fallback params
  this->get_parameter("decoding.temperature", this->wparams.temperature);
  this->get_parameter("decoding.max_initial_ts", this->wparams.max_initial_ts);
  this->get_parameter("decoding.length_penalty", this->wparams.length_penalty);
  this->get_parameter("decoding.temperature_inc",
                      this->wparams.temperature_inc);
  this->get_parameter("decoding.entropy_thold", this->wparams.entropy_thold);
  this->get_parameter("decoding.logprob_thold", this->wparams.logprob_thold);
  this->get_parameter("decoding.no_speech_thold",
                      this->wparams.no_speech_thold);

  // sampling params
  this->get_parameter("sampling.greedy_best_of", this->wparams.greedy.best_of);
  this->get_parameter("sampling.beam_search_beam_size",
                      this->wparams.beam_search.beam_size);
  this->get_parameter("sampling.beam_search_patience",
                      this->wparams.beam_search.patience);

  // diarization params
  this->get_parameter("diarization.tdrz_enable", this->wparams.tdrz_enable);

  // gpu / backend params
  this->get_parameter("transcription.n_processors", this->n_processors);
  this->get_parameter("gpu.flash_attn", this->cparams.flash_attn);
  this->get_parameter("gpu.enabled", this->cparams.use_gpu);
  this->get_parameter("gpu.device", this->cparams.gpu_device);

  // dtw params
  this->get_parameter("dtw.n_top", this->cparams.dtw_n_top);
  this->get_parameter("dtw.token_timestamps",
                      this->cparams.dtw_token_timestamps);
  this->get_parameter("dtw.aheads", dtw_aheads);

  // download model
  if (this->model.empty()) {
    this->model = whisper_utils::download_model(model_repo, model_filename);
  }

  // check threads number
  if (this->wparams.n_threads < 0) {
    this->wparams.n_threads = std::thread::hardware_concurrency();
  }

  // suppress_regex
  if (!this->suppress_regex.empty()) {
    this->wparams.suppress_regex = this->suppress_regex.c_str();
  }

  // initial_prompt
  if (!this->initial_prompt.empty()) {
    this->wparams.initial_prompt = this->initial_prompt.c_str();
  }

  // parse dtw_aheads
  if (dtw_aheads == "tiny") {
    this->cparams.dtw_aheads_preset = WHISPER_AHEADS_TINY;

  } else if (dtw_aheads == "tiny.en") {
    this->cparams.dtw_aheads_preset = WHISPER_AHEADS_TINY_EN;

  } else if (dtw_aheads == "base") {
    this->cparams.dtw_aheads_preset = WHISPER_AHEADS_BASE;

  } else if (dtw_aheads == "base.en") {
    this->cparams.dtw_aheads_preset = WHISPER_AHEADS_BASE_EN;

  } else if (dtw_aheads == "small") {
    this->cparams.dtw_aheads_preset = WHISPER_AHEADS_SMALL;

  } else if (dtw_aheads == "small.en") {
    this->cparams.dtw_aheads_preset = WHISPER_AHEADS_SMALL_EN;

  } else if (dtw_aheads == "medium") {
    this->cparams.dtw_aheads_preset = WHISPER_AHEADS_MEDIUM;

  } else if (dtw_aheads == "medium.en") {
    this->cparams.dtw_aheads_preset = WHISPER_AHEADS_MEDIUM_EN;

  } else if (dtw_aheads == "large.v1") {
    this->cparams.dtw_aheads_preset = WHISPER_AHEADS_LARGE_V1;

  } else if (dtw_aheads == "large.v2") {
    this->cparams.dtw_aheads_preset = WHISPER_AHEADS_LARGE_V2;

  } else if (dtw_aheads == "large.v3") {
    this->cparams.dtw_aheads_preset = WHISPER_AHEADS_LARGE_V3;

  } else if (dtw_aheads == "large.v3.turbo") {
    this->cparams.dtw_aheads_preset = WHISPER_AHEADS_LARGE_V3_TURBO;

  } else {
    this->cparams.dtw_aheads_preset = WHISPER_AHEADS_NONE;
  }

  RCLCPP_INFO(this->get_logger(), "[%s] Configured", this->get_name());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WhisperBaseNode::on_activate(const rclcpp_lifecycle::State &) {

  RCLCPP_INFO(this->get_logger(), "[%s] Activating...", this->get_name());

  // create whisper
  this->whisper = std::make_shared<Whisper>(
      this->model, this->openvino_encode_device, this->n_processors,
      this->cparams, this->wparams);

  this->activate_ros_interfaces();

  RCLCPP_INFO(this->get_logger(), "[%s] Activated", this->get_name());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WhisperBaseNode::on_deactivate(const rclcpp_lifecycle::State &) {

  RCLCPP_INFO(this->get_logger(), "[%s] Deactivating...", this->get_name());

  this->whisper.reset();
  this->whisper = nullptr;

  this->deactivate_ros_interfaces();

  RCLCPP_INFO(this->get_logger(), "[%s] Deactivated", this->get_name());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WhisperBaseNode::on_cleanup(const rclcpp_lifecycle::State &) {

  RCLCPP_INFO(this->get_logger(), "[%s] Cleaning up...", this->get_name());
  RCLCPP_INFO(this->get_logger(), "[%s] Cleaned up", this->get_name());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WhisperBaseNode::on_shutdown(const rclcpp_lifecycle::State &) {

  RCLCPP_INFO(this->get_logger(), "[%s] Shutting down...", this->get_name());
  RCLCPP_INFO(this->get_logger(), "[%s] Shut down", this->get_name());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

whisper_msgs::msg::Transcription
WhisperBaseNode::transcribe(const std::vector<float> &audio) {

  auto start_time = this->get_clock()->now();
  RCLCPP_INFO(this->get_logger(), "Transcribing");
  struct TranscriptionOutput result = this->whisper->transcribe(audio);
  std::string text = this->whisper->trim(result.text);
  auto end_time = this->get_clock()->now();

  RCLCPP_INFO(this->get_logger(), "Text heard: %s", text.c_str());
  whisper_msgs::msg::Transcription msg;
  msg.text = text;
  msg.audio_time = static_cast<float>(audio.size()) / WHISPER_SAMPLE_RATE;
  msg.transcription_time = (end_time - start_time).seconds();

  return msg;
}
