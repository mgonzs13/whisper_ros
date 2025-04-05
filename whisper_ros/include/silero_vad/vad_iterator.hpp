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

#ifndef SILERO_VAD__VAD_ITERATOR_HPP
#define SILERO_VAD__VAD_ITERATOR_HPP

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "onnxruntime_cxx_api.h"
#include "silero_vad/timestamp.hpp"

namespace silero_vad {

/// @class VadIterator
/// @brief Implements a Voice Activity Detection (VAD) iterator using an ONNX
/// model.
///
/// This class provides methods to load a pre-trained ONNX VAD model, process
/// audio data, and predict the presence of speech. It manages the model state
/// and handles input/output tensors for inference.
class VadIterator {

public:
  /// @brief Constructs a VadIterator object.
  ///
  /// @param model_path Path to the ONNX model file.
  /// @param sample_rate The audio sample rate in Hz (default: 16000).
  /// @param frame_size_ms Size of the audio frame in milliseconds (default:
  /// 32).
  /// @param threshold The threshold for speech detection (default: 0.5).
  /// @param min_silence_ms Minimum silence duration in milliseconds to mark the
  /// end of speech (default: 100).
  /// @param speech_pad_ms Additional padding in milliseconds added to speech
  /// segments (default: 30).
  VadIterator(const std::string &model_path, int sample_rate = 16000,
              int frame_size_ms = 32, float threshold = 0.5f,
              int min_silence_ms = 100, int speech_pad_ms = 30,
              bool use_cuda = false);

  /// @brief Resets the internal state of the model.
  ///
  /// Clears the state, context, and resets internal flags related to speech
  /// detection.
  void reset_states();

  /// @brief Processes audio data and predicts speech segments.
  ///
  /// @param data A vector of audio samples (single-channel, float values).
  /// @return A Timestamp object containing start and end times of detected
  /// speech, or -1 for inactive values.
  Timestamp predict(const std::vector<float> &data);

private:
  /// ONNX Runtime environment.
  Ort::Env env;
  /// ONNX session options.
  Ort::SessionOptions session_options;
  /// ONNX session for running inference.
  std::shared_ptr<Ort::Session> session;
  /// Memory allocator for ONNX runtime.
  Ort::AllocatorWithDefaultOptions allocator;
  /// Memory info for tensor allocation.
  Ort::MemoryInfo memory_info =
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeCPU);

  // Indicates whether to use CUDA for inference.
  bool use_cuda;

  /// Detection threshold for speech probability.
  float threshold;
  /// Audio sample rate in Hz.
  int sample_rate;
  /// Samples per millisecond.
  int sr_per_ms;
  /// Number of samples in a single frame.
  int64_t window_size_samples;
  /// Padding in samples added to speech segments.
  int speech_pad_samples;
  /// Minimum silence duration in samples to mark the end of speech.
  unsigned int min_silence_samples;
  /// Size of the context buffer.
  int context_size;

  /// Indicates whether speech has been detected.
  bool triggered = false;
  /// Temporary end position during silence detection.
  unsigned int temp_end = 0;
  /// Current sample position in the input stream.
  unsigned int current_sample = 0;
  /// End sample of the last speech segment.
  int prev_end = 0;
  /// Start sample of the next speech segment.
  int next_start = 0;

  /// ONNX model input tensors.
  std::vector<Ort::Value> ort_inputs;

  /// Names of the input nodes in the ONNX model.
  std::vector<const char *> input_node_names = {"input", "state", "sr"};

  /// Input buffer for audio data and context.
  std::vector<float> input;

  /// Context buffer storing past audio samples.
  std::vector<float> context;

  /// Internal state of the model.
  std::vector<float> state;

  /// Sample rate tensor.
  std::vector<int64_t> sr;

  /// Dimensions for the input tensor.
  int64_t input_node_dims[2] = {};

  /// Dimensions for the state tensor.
  const int64_t state_node_dims[3] = {2, 1, 128};

  /// Dimensions for the sample rate tensor.
  const int64_t sr_node_dims[1] = {1};

  /// ONNX model output tensors.
  std::vector<Ort::Value> ort_outputs;

  /// Names of the output nodes in the ONNX model.
  std::vector<const char *> output_node_names = {"output", "stateN"};

  /// @brief Initializes the ONNX model session.
  ///
  /// @param model_path Path to the ONNX model file.
  /// @throws std::runtime_error If the ONNX session initialization fails.
  void init_onnx_model(const std::string &model_path);
};

} // namespace silero_vad

#endif
