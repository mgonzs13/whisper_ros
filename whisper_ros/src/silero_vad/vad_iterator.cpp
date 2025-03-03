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

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "silero_vad/vad_iterator.hpp"
#include "whisper_utils/logs.hpp"

using namespace silero_vad;

VadIterator::VadIterator(const std::string &model_path, int sample_rate,
                         int frame_size_ms, float threshold, int min_silence_ms,
                         int speech_pad_ms, bool use_cuda)
    : env(ORT_LOGGING_LEVEL_WARNING, "VadIterator"), use_cuda(use_cuda),
      threshold(threshold), sample_rate(sample_rate),
      sr_per_ms(sample_rate / 1000),
      window_size_samples(frame_size_ms * sr_per_ms),
      speech_pad_samples(sr_per_ms * speech_pad_ms),
      min_silence_samples(sr_per_ms * min_silence_ms),
      context_size(sample_rate == 16000 ? 64 : 32), context(context_size, 0.0f),
      state(2 * 1 * 128, 0.0f), sr(1, sample_rate) {

  this->input_node_dims[0] = 1;
  this->input_node_dims[1] = this->context_size + this->window_size_samples;
  this->input.reserve(this->context_size + this->window_size_samples);

  try {
    this->init_onnx_model(model_path);
  } catch (const std::exception &e) {
    WHISPER_LOG_ERROR("Failed to initialize ONNX model: %s", e.what());
    return;
  }

  WHISPER_LOG_INFO("SileroVAD Iterator started");
}

void VadIterator::init_onnx_model(const std::string &model_path) {
  this->session_options.SetIntraOpNumThreads(1);
  this->session_options.SetInterOpNumThreads(1);
  this->session_options.SetGraphOptimizationLevel(
      GraphOptimizationLevel::ORT_ENABLE_ALL);

  std::vector<std::string> providers = Ort::GetAvailableProviders();
  bool cuda_available = false;

  for (const auto &provider : providers) {
    if (provider == "CUDAExecutionProvider") {
      cuda_available = true;
      break;
    }
  }

  if (this->use_cuda && cuda_available) {
    OrtCUDAProviderOptions cudaOption;
    WHISPER_LOG_INFO("Using CUDA provider");
    this->session_options.AppendExecutionProvider_CUDA(cudaOption);
  } else if (this->use_cuda && !cuda_available) {
    WHISPER_LOG_WARN("CUDA provider not available, using CPU provider");
  } else {
    WHISPER_LOG_INFO("Using CPU provider");
  }

  try {
    this->session = std::make_shared<Ort::Session>(
        this->env, model_path.c_str(), this->session_options);
  } catch (const std::exception &e) {
    throw std::runtime_error("Failed to create ONNX session: " +
                             std::string(e.what()));
  }
}

void VadIterator::reset_states() {
  std::fill(this->state.begin(), this->state.end(), 0.0f);
  std::fill(this->context.begin(), this->context.end(), 0.0f);
  this->triggered = false;
  this->temp_end = 0;
  this->current_sample = 0;
}

Timestamp VadIterator::predict(const std::vector<float> &data) {

  WHISPER_LOG_DEBUG("Processing audio data");

  // Pre-fill input with context
  this->input.clear();
  this->input.insert(this->input.end(), this->context.begin(),
                     this->context.end());
  this->input.insert(this->input.end(), data.begin(), data.end());

  // Create input tensors
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
  try {
    this->ort_outputs = session->Run(
        Ort::RunOptions{nullptr}, this->input_node_names.data(),
        this->ort_inputs.data(), this->ort_inputs.size(),
        this->output_node_names.data(), this->output_node_names.size());
  } catch (const std::exception &e) {
    WHISPER_LOG_ERROR("ONNX inference failed: %s", e.what());
    return Timestamp(-1, -1, 0.0f);
  }

  // Process output
  float speech_prob = this->ort_outputs[0].GetTensorMutableData<float>()[0];
  float *updated_state = this->ort_outputs[1].GetTensorMutableData<float>();
  std::copy(updated_state, updated_state + this->state.size(),
            this->state.begin());
  WHISPER_LOG_DEBUG("Speech probability %f", speech_prob);

  // Update context with the last 64 samples of data
  this->context.assign(data.end() - context_size, data.end());

  // Handle result
  this->current_sample += this->window_size_samples;

  if (speech_prob >= this->threshold) {
    if (this->temp_end != 0) {
      this->temp_end = 0;
    }

    if (!this->triggered) {
      int start_timestwamp = this->current_sample - this->speech_pad_samples -
                             this->window_size_samples;
      this->triggered = true;
      WHISPER_LOG_DEBUG("Speech starts at %d", start_timestwamp);
      return Timestamp(start_timestwamp, -1, speech_prob);
    }
  }

  if (speech_prob < this->threshold - 0.15 && this->triggered) {
    if (this->temp_end == 0) {
      this->temp_end = this->current_sample;
    }

    if (this->current_sample - this->temp_end >= this->min_silence_samples) {
      int end_timestamp =
          this->temp_end + this->speech_pad_samples - this->window_size_samples;
      this->triggered = false;
      this->temp_end = 0;
      WHISPER_LOG_DEBUG("Speech ends at %d", end_timestamp);
      return Timestamp(-1, end_timestamp, speech_prob);
    }
  }

  return Timestamp(-1, -1, speech_prob);
}
