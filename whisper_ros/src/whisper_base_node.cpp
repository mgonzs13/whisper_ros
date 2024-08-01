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

WhisperBaseNode::WhisperBaseNode() : rclcpp::Node("whisper_node") {

  std::string model;
  std::string openvino_encode_device;
  int n_processors;

  std::string sampling_strategy;
  struct whisper_context_params cparams = whisper_context_default_params();

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

  // get sampling method and create default params
  this->get_parameter("sampling_strategy", sampling_strategy);

  struct whisper_full_params wparams;

  if (sampling_strategy == "greedy") {
    wparams = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);
  } else if (sampling_strategy == "beam_search") {
    wparams = whisper_full_default_params(WHISPER_SAMPLING_BEAM_SEARCH);
  }

  // get params
  this->get_parameter("model", model);
  this->get_parameter("openvino_encode_device", openvino_encode_device);

  this->get_parameter("n_threads", wparams.n_threads);
  this->get_parameter("n_max_text_ctx", wparams.n_max_text_ctx);
  this->get_parameter("offset_ms", wparams.offset_ms);
  this->get_parameter("duration_ms", wparams.duration_ms);

  this->get_parameter("translate", wparams.translate);
  this->get_parameter("no_context", wparams.no_context);
  this->get_parameter("no_timestamps", wparams.no_timestamps);
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

  this->get_parameter("audio_ctx", wparams.audio_ctx);
  this->get_parameter("tinydiarize", wparams.tdrz_enable);

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

  this->get_parameter("greedy_best_of", wparams.greedy.best_of);
  this->get_parameter("beam_search_beam_size", wparams.beam_search.beam_size);
  this->get_parameter("beam_search_patience", wparams.beam_search.patience);

  this->get_parameter("n_processors", n_processors);
  this->get_parameter("use_gpu", cparams.use_gpu);
  this->get_parameter("gpu_device", cparams.gpu_device);

  // check threads number
  if (wparams.n_threads < 0) {
    wparams.n_threads = std::thread::hardware_concurrency();
  }

  // create whisper
  this->whisper = std::make_shared<Whisper>(model, openvino_encode_device,
                                            n_processors, cparams, wparams);
}
