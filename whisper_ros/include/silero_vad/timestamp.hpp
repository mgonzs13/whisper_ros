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

#ifndef SILERO_VAD__TIMESTAMPT_HPP
#define SILERO_VAD__TIMESTAMPT_HPP

#include <string>

namespace silero_vad {

/// @class Timestamp
/// @brief Represents a time interval with speech probability.
class Timestamp {

public:
  /// The start time of the interval, in milliseconds.
  int start;

  /// The end time of the interval, in milliseconds.
  int end;

  /// The probability of speech detected in the interval, ranging from 0 to 1.
  float speech_prob;

  /// @brief Constructs a `Timestamp` object.
  /// @param start The start time of the interval (default: -1).
  /// @param end The end time of the interval (default: -1).
  /// @param speech_prob The speech probability (default: 0).
  Timestamp(int start = -1, int end = -1, float speech_prob = 0);

  /// @brief Assigns the values of another `Timestamp` to this instance.
  /// @param other The `Timestamp` to copy from.
  /// @return A reference to this `Timestamp`.
  Timestamp &operator=(const Timestamp &other);

  /// @brief Compares two `Timestamp` objects for equality.
  /// @param other The `Timestamp` to compare with.
  /// @return `true` if the start and end times are equal; `false` otherwise.
  bool operator==(const Timestamp &other) const;

  /// @brief Converts the `Timestamp` object to a string representation.
  /// @return A string representing the `Timestamp` in the format
  /// `{start:...,end:...,prob:...}`.
  std::string to_string() const;
};

} // namespace silero_vad

#endif