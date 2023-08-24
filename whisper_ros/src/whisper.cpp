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

#include <thread>

#include "whisper_ros/whisper.hpp"

using namespace whisper_ros;

Whisper::Whisper(const std::string &model,
                 const std::string &openvino_encode_device,
                 const whisper_full_params &wparams)
    : wparams(wparams) {

  if (whisper_lang_id(wparams.language) == -1) {
    fprintf(stderr, "Unknown language '%s'\n", wparams.language);
    exit(0);
  }

  // init whisper
  this->ctx = whisper_init_from_file(model.c_str());

  if (this->ctx == nullptr) {
    fprintf(stderr, "error: failed to initialize whisper context\n");
  }

  if (!whisper_is_multilingual(this->ctx)) {
    if (std::string(this->wparams.language) != "en" ||
        this->wparams.translate) {
      this->wparams.language = "en";
      this->wparams.translate = false;
      fprintf(stderr, "Model is not multilingual, ignoring language and "
                      "translation options\n");
    }
  }

  // initialize openvino encoder
  // this has no effect on whisper.cpp builds
  // that don't have OpenVINO configured
  whisper_ctx_init_openvino_encoder(this->ctx, nullptr,
                                    openvino_encode_device.c_str(), nullptr);

  fprintf(stderr,
          "Processing, %d threads, lang = %s, task = %s, timestamps = %d ...\n",
          this->wparams.n_threads, this->wparams.language,
          this->wparams.translate ? "translate" : "transcribe",
          this->wparams.print_timestamps ? 0 : 1);

  fprintf(stderr, "system_info: n_threads = %d / %d | %s\n",
          this->wparams.n_threads, std::thread::hardware_concurrency(),
          whisper_print_system_info());
}

Whisper::~Whisper() { whisper_free(this->ctx); }

transcription_output Whisper::transcribe(const std::vector<float> &pcmf32) {

  int prob_n = 0;
  transcription_output result;
  result.text = "";
  result.prob = 0.0f;

  if (whisper_full(this->ctx, this->wparams, pcmf32.data(), pcmf32.size())) {
    return result;
  }

  const int n_segments = whisper_full_n_segments(this->ctx);
  for (int i = 0; i < n_segments; ++i) {
    const char *text = whisper_full_get_segment_text(this->ctx, i);

    result.text += text;

    const int n_tokens = whisper_full_n_tokens(this->ctx, i);
    for (int j = 0; j < n_tokens; ++j) {
      const auto token = whisper_full_get_token_data(this->ctx, i, j);

      result.prob += token.p;
      ++prob_n;
    }
  }

  if (prob_n > 0) {
    result.prob /= prob_n;
  }

  return result;
}
