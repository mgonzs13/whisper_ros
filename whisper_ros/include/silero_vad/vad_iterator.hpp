// MIT License
//
// Copyright (c) 2024  Miguel Ángel González Santamarta
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

#ifndef SILERO_VAD__VAD_ITERATOR_HPP
#define SILERO_VAD__VAD_ITERATOR_HPP

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "onnxruntime_cxx_api.h"
#include "silero_vad/timestamp.hpp"

namespace silero_vad {

class VadIterator {

public:
  VadIterator(const std::string &model_path, int sample_rate = 16000,
              int frame_size_ms = 32, float threshold = 0.5f,
              int min_silence_ms = 100, int speech_pad_ms = 30);

  void reset_states();
  Timestamp predict(const std::vector<float> &data);

private:
  Ort::Env env;
  Ort::SessionOptions session_options;
  std::shared_ptr<Ort::Session> session;
  Ort::AllocatorWithDefaultOptions allocator;
  Ort::MemoryInfo memory_info =
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeCPU);

  // Model configuration
  float threshold;
  int sample_rate;
  int sr_per_ms;
  int64_t window_size_samples;
  int speech_pad_samples;
  unsigned int min_silence_samples;
  int context_size;

  // Model state
  bool triggered = false;
  unsigned int temp_end = 0;
  unsigned int current_sample = 0;
  int prev_end = 0;
  int next_start = 0;

  std::vector<Ort::Value> ort_inputs;
  std::vector<const char *> input_node_names = {"input", "state", "sr"};

  std::vector<float> input;
  std::vector<float> context;
  std::vector<float> state;
  std::vector<int64_t> sr;

  int64_t input_node_dims[2] = {};
  const int64_t state_node_dims[3] = {2, 1, 128};
  const int64_t sr_node_dims[1] = {1};

  std::vector<Ort::Value> ort_outputs;
  std::vector<const char *> output_node_names = {"output", "stateN"};

  void init_onnx_model(const std::string &model_path);
};

} // namespace silero_vad

#endif