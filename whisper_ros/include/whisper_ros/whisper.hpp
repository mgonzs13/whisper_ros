// MIT License
//
// Copyright (c) 2023  Miguel Ángel González Santamarta
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

#ifndef WHISPER_ROS__WHISPER_HPP
#define WHISPER_ROS__WHISPER_HPP

#include <string>
#include <vector>

#include "grammar-parser.h"
#include "whisper.h"

// whisper logs
#define WHISPER_LOG_ERROR(text, ...)                                           \
  fprintf(stderr, "[ERROR] " text "\n", ##__VA_ARGS__)
#define WHISPER_LOG_WARN(text, ...)                                            \
  fprintf(stderr, "[WARN] " text "\n", ##__VA_ARGS__)
#define WHISPER_LOG_INFO(text, ...)                                            \
  fprintf(stderr, "[INFO] " text "\n", ##__VA_ARGS__)

struct transcription_output {
  std::string text;
  float prob;
};

namespace whisper_ros {

class Whisper {

public:
  Whisper(const std::string &model, const std::string &openvino_encode_device,
          int n_processors, const struct whisper_context_params &cparams,
          const struct whisper_full_params &wparams);
  ~Whisper();

  struct transcription_output transcribe(const std::vector<float> &pcmf32);
  std::string trim(const std::string &s);
  std::string timestamp_to_str(int64_t t, bool comma = false);

  bool set_grammar(const std::string grammar, const std::string start_rule,
                   float grammar_penalty);
  void reset_grammar();
  void set_init_prompt(const std::string prompt);
  void reset_init_prompt();

protected:
  int n_processors;
  struct whisper_full_params wparams;

  struct whisper_context *ctx;
  grammar_parser::parse_state grammar_parsed;
  std::vector<const whisper_grammar_element *> grammar_rules;
};

} // namespace whisper_ros

#endif