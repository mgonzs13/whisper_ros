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

#ifndef WHISPER_ROS__WHISPER_HPP
#define WHISPER_ROS__WHISPER_HPP

#include <string>
#include <vector>

#include "grammar-parser.h"
#include "whisper.h"

/// Represents the result of a transcription operation.
struct TranscriptionOutput {
  /// The transcribed text.
  std::string text;

  /// The confidence probability of the transcription.
  float prob;
};

namespace whisper_ros {

/// Class for performing speech-to-text transcription using the Whisper model.
class Whisper {

public:
  /// Constructs a Whisper object with the specified model and parameters.
  /// @param model The path to the Whisper model file.
  /// @param openvino_encode_device The OpenVINO device used for encoding.
  /// @param n_processors Number of processors to use for parallel processing.
  /// @param cparams Whisper context parameters.
  /// @param wparams Whisper full parameters.
  Whisper(const std::string &model, const std::string &openvino_encode_device,
          int n_processors, const struct whisper_context_params &cparams,
          const struct whisper_full_params &wparams);

  /// Destructor to clean up resources used by the Whisper object.
  ~Whisper();

  /// Transcribes the given audio data.
  /// @param pcmf32 A vector of audio samples in 32-bit float format.
  /// @return A TranscriptionOutput structure containing the transcription text
  /// and confidence probability.
  struct TranscriptionOutput transcribe(const std::vector<float> &pcmf32);

  /// Trims leading and trailing whitespace from the input string.
  /// @param s The string to trim.
  /// @return The trimmed string.
  std::string trim(const std::string &s);

  /// Converts a timestamp to a string format.
  /// @param t The timestamp in 10 ms units.
  /// @param comma If true, use a comma as the decimal separator; otherwise, use
  /// a period.
  /// @return The formatted timestamp as a string.
  std::string timestamp_to_str(int64_t t, bool comma = false);

  /// Sets a grammar for transcription with a starting rule and penalty.
  /// @param grammar The grammar rules as a string.
  /// @param start_rule The starting rule for the grammar.
  /// @param grammar_penalty A penalty factor for grammar violations.
  /// @return True if the grammar is set successfully; false otherwise.
  bool set_grammar(const std::string grammar, const std::string start_rule,
                   float grammar_penalty);

  /// Resets the grammar to its default state.
  void reset_grammar();

  /// Sets an initial prompt for transcription.
  /// @param prompt The initial prompt text.
  void set_init_prompt(const std::string prompt);

  /// Resets the initial prompt to its default state.
  void reset_init_prompt();

protected:
  /// Number of processors used for parallel processing.
  int n_processors;

  /// Parameters used for full transcription tasks.
  struct whisper_full_params wparams;

  /// The Whisper context.
  struct whisper_context *ctx;

  /// Parsed grammar state.
  grammar_parser::parse_state grammar_parsed;

  /// Grammar rules derived from the parsed grammar.
  std::vector<const whisper_grammar_element *> grammar_rules;
};

} // namespace whisper_ros

#endif
