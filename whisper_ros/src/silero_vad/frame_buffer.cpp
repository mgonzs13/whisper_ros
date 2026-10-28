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

#include "silero_vad/frame_buffer.hpp"

#include <stdexcept>

namespace silero_vad {

FrameBuffer::FrameBuffer(std::size_t frame_size) : frame_size_(frame_size) {
  if (this->frame_size_ == 0) {
    throw std::invalid_argument("FrameBuffer frame size must be positive");
  }
}

void FrameBuffer::push(const std::vector<float> &samples) {
  this->samples_.insert(this->samples_.end(), samples.begin(), samples.end());
}

bool FrameBuffer::pop(std::vector<float> &frame) {
  if (this->samples_.size() < this->frame_size_) {
    return false;
  }

  frame.assign(this->samples_.begin(),
               this->samples_.begin() + this->frame_size_);
  this->samples_.erase(this->samples_.begin(),
                       this->samples_.begin() + this->frame_size_);
  return true;
}

void FrameBuffer::clear() { this->samples_.clear(); }

std::size_t FrameBuffer::pending() const { return this->samples_.size(); }

} // namespace silero_vad
