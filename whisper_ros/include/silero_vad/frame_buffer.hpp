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

#ifndef SILERO_VAD__FRAME_BUFFER_HPP_
#define SILERO_VAD__FRAME_BUFFER_HPP_

#include <cstddef>
#include <deque>
#include <vector>

namespace silero_vad {

/// @brief Accumulates arbitrary-length audio chunks into fixed-size frames.
///
/// Audio messages from different publishers (for example an AEC node) may
/// have a different chunk size than the frame size expected by Silero VAD.
/// This buffer lets the node feed the model complete frames without dropping
/// or zero-padding the tail of every message.
class FrameBuffer {
public:
  /// @param frame_size Number of samples per frame. Must be positive.
  explicit FrameBuffer(std::size_t frame_size);

  /// @brief Appends samples to the pending buffer.
  void push(const std::vector<float> &samples);

  /// @brief Copies the oldest complete frame into @p frame and removes it.
  /// @return False when fewer than frame_size samples are pending.
  bool pop(std::vector<float> &frame);

  /// @brief Drops all pending samples.
  void clear();

  /// @brief Number of samples currently waiting for a complete frame.
  std::size_t pending() const;

private:
  std::size_t frame_size_;
  std::deque<float> samples_;
};

} // namespace silero_vad

#endif // SILERO_VAD__FRAME_BUFFER_HPP_
