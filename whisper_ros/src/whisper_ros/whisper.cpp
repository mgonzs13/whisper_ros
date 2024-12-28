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

#include <regex>
#include <thread>

#include "grammar-parser.h"
#include "whisper_ros/whisper.hpp"
#include "whisper_utils/logs.hpp"

using namespace whisper_ros;

Whisper::Whisper(const std::string &model,
                 const std::string &openvino_encode_device, int n_processors,
                 const struct whisper_context_params &cparams,
                 const whisper_full_params &wparams)
    : n_processors(n_processors), wparams(wparams) {

  if (whisper_lang_id(wparams.language) == -1) {
    WHISPER_LOG_ERROR("Unknown language '%s'\n", wparams.language);
  }

  // init whisper
  this->ctx = whisper_init_from_file_with_params(model.c_str(), cparams);

  if (this->ctx == nullptr) {
    WHISPER_LOG_ERROR("Failed to initialize whisper context\n");
  }

  if (!whisper_is_multilingual(this->ctx)) {
    if (std::string(this->wparams.language) != "en" ||
        this->wparams.translate) {
      this->wparams.language = "en";
      this->wparams.translate = false;
      WHISPER_LOG_WARN("Model is not multilingual, ignoring language and "
                       "translation options\n");
    }
  }

  // initialize openvino encoder
  // this has no effect on whisper.cpp builds
  // that don't have OpenVINO configured
  whisper_ctx_init_openvino_encoder(this->ctx, nullptr,
                                    openvino_encode_device.c_str(), nullptr);

  WHISPER_LOG_INFO("Processing, %d threads, lang = %s, task = %s",
                   this->wparams.n_threads, this->wparams.language,
                   this->wparams.translate ? "translate" : "transcribe");

  WHISPER_LOG_INFO("system_info: n_threads = %d / %d | %s\n",
                   this->wparams.n_threads, std::thread::hardware_concurrency(),
                   whisper_print_system_info());
}

Whisper::~Whisper() { whisper_free(this->ctx); }

struct TranscriptionOutput
Whisper::transcribe(const std::vector<float> &pcmf32) {

  WHISPER_LOG_DEBUG("Starting transcription");

  int prob_n = 0;
  struct TranscriptionOutput result;
  result.text = "";
  result.prob = 0.0f;

  if (whisper_full_parallel(this->ctx, this->wparams, pcmf32.data(),
                            pcmf32.size(), this->n_processors) != 0) {
    WHISPER_LOG_ERROR("Error in whisper_full_parallel");
    return result;
  }

  const int n_segments = whisper_full_n_segments(this->ctx);

  for (int i = 0; i < n_segments; ++i) {
    const char *text = whisper_full_get_segment_text(this->ctx, i);
    result.text += text;

    int64_t t0 = whisper_full_get_segment_t0(this->ctx, i);
    int64_t t1 = whisper_full_get_segment_t1(this->ctx, i);
    WHISPER_LOG_INFO("[%s --> %s]: %s", this->timestamp_to_str(t0).c_str(),
                     this->timestamp_to_str(t1).c_str(), text);

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

std::string Whisper::trim(const std::string &s) {
  std::regex e("^\\s+|\\s+$");
  return std::regex_replace(s, e, "");
}

std::string Whisper::timestamp_to_str(int64_t t, bool comma) {
  int64_t msec = t * 10;
  int64_t hr = msec / (1000 * 60 * 60);
  msec = msec - hr * (1000 * 60 * 60);
  int64_t min = msec / (1000 * 60);
  msec = msec - min * (1000 * 60);
  int64_t sec = msec / 1000;
  msec = msec - sec * 1000;

  char buf[32];
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d%s%03d", (int)hr, (int)min,
           (int)sec, comma ? "," : ".", (int)msec);

  return std::string(buf);
}

bool Whisper::set_grammar(const std::string grammar,
                          const std::string start_rule, float grammar_penalty) {

  WHISPER_LOG_DEBUG("Setting new grammar");
  this->grammar_parsed = grammar_parser::parse(grammar.c_str());

  if (this->grammar_parsed.rules.empty()) {
    WHISPER_LOG_ERROR("Error setting the grammar");
    return false;
  }

  this->grammar_rules = this->grammar_parsed.c_rules();

  this->wparams.grammar_rules = this->grammar_rules.data();
  this->wparams.n_grammar_rules = this->grammar_rules.size();
  this->wparams.i_start_rule =
      this->grammar_parsed.symbol_ids.at(start_rule.c_str());
  this->wparams.grammar_penalty = grammar_penalty;

  return true;
}

void Whisper::reset_grammar() {
  WHISPER_LOG_DEBUG("Resetting grammar");
  this->wparams.grammar_rules = nullptr;
  this->wparams.n_grammar_rules = 0;
  this->wparams.i_start_rule = 0;
  this->wparams.grammar_penalty = 100.0f;
}

void Whisper::set_init_prompt(const std::string prompt) {
  WHISPER_LOG_DEBUG("Resetting initial prompt");
  this->wparams.initial_prompt = prompt.c_str();
}

void Whisper::reset_init_prompt() {
  WHISPER_LOG_DEBUG("Resetting initial prompt");
  this->wparams.initial_prompt = "";
}
