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

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "silero_vad/vad_iterator.hpp"

using namespace silero_vad;

VadIterator::VadIterator(const std::string &model_path, int sample_rate,
                         int frame_size_ms, float threshold, int min_silence_ms,
                         int speech_pad_ms, int min_speech_ms,
                         float max_speech_s)
    : env(ORT_LOGGING_LEVEL_WARNING, "VadIterator"), threshold(threshold),
      sample_rate(sample_rate), sr_per_ms(sample_rate / 1000),
      window_size_samples(frame_size_ms * sr_per_ms),
      min_speech_samples(sr_per_ms * min_speech_ms),
      speech_pad_samples(sr_per_ms * speech_pad_ms),
      max_speech_samples(sample_rate * max_speech_s - window_size_samples -
                         2 * speech_pad_samples),
      min_silence_samples(sr_per_ms * min_silence_ms),
      min_silence_samples_at_max_speech(sr_per_ms * 98),
      state(2 * 1 * 128, 0.0f), sr(1, sample_rate), context(64, 0.0f) {

  // this->input.resize(window_size_samples);
  this->input_node_dims[0] = 1;
  this->input_node_dims[1] = window_size_samples;
  this->init_onnx_model(model_path);
}

void VadIterator::init_onnx_model(const std::string &model_path) {
  this->session_options.SetIntraOpNumThreads(1);
  this->session_options.SetInterOpNumThreads(1);
  this->session_options.SetGraphOptimizationLevel(
      GraphOptimizationLevel::ORT_ENABLE_ALL);
  this->session = std::make_shared<Ort::Session>(this->env, model_path.c_str(),
                                                 this->session_options);
}

void VadIterator::reset_states() {
  std::fill(this->state.begin(), this->state.end(), 0.0f);
  std::fill(this->context.begin(), this->context.end(), 0.0f);
  this->triggered = false;
  this->temp_end = 0;
  this->current_sample = 0;
}

Timestamp VadIterator::predict(const std::vector<float> &data) {
  // Create input tensors
  this->input.clear();
  for (auto ele : this->context) {
    this->input.push_back(ele);
  }

  for (auto ele : data) {
    this->input.push_back(ele);
  }

  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
      this->memory_info, this->input.data(), this->input.size(),
      this->input_node_dims, 2);
  Ort::Value state_tensor = Ort::Value::CreateTensor<float>(
      this->memory_info, this->state.data(), this->state.size(),
      this->state_node_dims, 3);
  Ort::Value sr_tensor =
      Ort::Value::CreateTensor<int64_t>(this->memory_info, this->sr.data(),
                                        this->sr.size(), this->sr_node_dims, 1);

  // Clear and add inputs
  this->ort_inputs.clear();
  this->ort_inputs.emplace_back(std::move(input_tensor));
  this->ort_inputs.emplace_back(std::move(state_tensor));
  this->ort_inputs.emplace_back(std::move(sr_tensor));

  // Run inference
  this->ort_outputs = this->session->Run(
      Ort::RunOptions{nullptr}, this->input_node_names.data(),
      this->ort_inputs.data(), this->ort_inputs.size(),
      this->output_node_names.data(), this->output_node_names.size());

  // Process output
  float speech_prob = this->ort_outputs[0].GetTensorMutableData<float>()[0];
  float *updated_state = this->ort_outputs[1].GetTensorMutableData<float>();
  std::copy(updated_state, updated_state + this->state.size(),
            this->state.begin());

  for (int i = 63; i >= 0; i--) {
    this->context.push_back(data.at(data.size() - i));
  }

  // Handle result
  this->current_sample += this->window_size_samples;

  if (speech_prob >= this->threshold) {
    if (this->temp_end != 0) {
      this->temp_end = 0;
    }

    if (!this->triggered) {
      this->triggered = true;
      return Timestamp(this->current_sample - this->speech_pad_samples -
                           this->window_size_samples,
                       -1, speech_prob);
    }

  } else if (speech_prob < this->threshold - 0.15 && this->triggered) {
    if (this->temp_end == 0) {
      this->temp_end = this->current_sample;
    }

    if (this->current_sample - this->temp_end >= this->min_silence_samples) {
      this->temp_end = 0;
      this->triggered = false;
      return Timestamp(-1,
                       this->temp_end + this->speech_pad_samples -
                           this->window_size_samples,
                       speech_prob);
    }
  }

  return Timestamp(-1, -1, speech_prob);
}
