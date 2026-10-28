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

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include "silero_vad/frame_buffer.hpp"

using silero_vad::FrameBuffer;

TEST(FrameBuffer, EmitsFixedSizeFramesFromPartialChunks) {
  FrameBuffer buffer(512);
  std::vector<float> frame;

  buffer.push(std::vector<float>(320, 1.0f));
  EXPECT_FALSE(buffer.pop(frame));

  buffer.push(std::vector<float>(320, 2.0f));
  ASSERT_TRUE(buffer.pop(frame));
  ASSERT_EQ(frame.size(), 512u);
  EXPECT_FLOAT_EQ(frame[0], 1.0f);
  EXPECT_FLOAT_EQ(frame[319], 1.0f);
  EXPECT_FLOAT_EQ(frame[320], 2.0f);
  EXPECT_FLOAT_EQ(frame[511], 2.0f);
  EXPECT_FALSE(buffer.pop(frame));

  buffer.push(std::vector<float>(384, 3.0f));
  ASSERT_TRUE(buffer.pop(frame));
  ASSERT_EQ(frame.size(), 512u);
  EXPECT_FLOAT_EQ(frame[0], 2.0f);
  EXPECT_FLOAT_EQ(frame[127], 2.0f);
  EXPECT_FLOAT_EQ(frame[128], 3.0f);
  EXPECT_FLOAT_EQ(frame[511], 3.0f);
  EXPECT_FALSE(buffer.pop(frame));
}

TEST(FrameBuffer, ClearDropsPendingSamples) {
  FrameBuffer buffer(4);
  std::vector<float> frame;

  buffer.push({1.0f, 2.0f, 3.0f});
  buffer.clear();
  buffer.push({4.0f, 5.0f, 6.0f, 7.0f});

  ASSERT_TRUE(buffer.pop(frame));
  ASSERT_EQ(frame.size(), 4u);
  EXPECT_FLOAT_EQ(frame[0], 4.0f);
  EXPECT_FLOAT_EQ(frame[3], 7.0f);
}

TEST(FrameBuffer, EmitsMultipleFramesFromOnePush) {
  FrameBuffer buffer(4);
  std::vector<float> frame;

  buffer.push({1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f});

  ASSERT_TRUE(buffer.pop(frame));
  EXPECT_EQ(frame, (std::vector<float>{1.0f, 2.0f, 3.0f, 4.0f}));
  ASSERT_TRUE(buffer.pop(frame));
  EXPECT_EQ(frame, (std::vector<float>{5.0f, 6.0f, 7.0f, 8.0f}));
  EXPECT_FALSE(buffer.pop(frame));
  EXPECT_EQ(buffer.pending(), 1u);

  buffer.push({10.0f, 11.0f, 12.0f});
  ASSERT_TRUE(buffer.pop(frame));
  EXPECT_EQ(frame, (std::vector<float>{9.0f, 10.0f, 11.0f, 12.0f}));
  EXPECT_FALSE(buffer.pop(frame));
  EXPECT_EQ(buffer.pending(), 0u);
}

TEST(FrameBuffer, EmptyPushKeepsPendingSamples) {
  FrameBuffer buffer(4);
  std::vector<float> frame;

  buffer.push({1.0f, 2.0f, 3.0f});
  buffer.push({});
  EXPECT_FALSE(buffer.pop(frame));
  EXPECT_EQ(buffer.pending(), 3u);

  buffer.push({4.0f});
  ASSERT_TRUE(buffer.pop(frame));
  EXPECT_EQ(frame, (std::vector<float>{1.0f, 2.0f, 3.0f, 4.0f}));
}

TEST(FrameBuffer, RejectsZeroFrameSize) {
  EXPECT_THROW(FrameBuffer(0), std::invalid_argument);
}
