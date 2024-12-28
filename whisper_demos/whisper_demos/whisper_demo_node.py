#!/usr/bin/env python3

# MIT License

# Copyright (c) 2023 Miguel Ángel González Santamarta

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from whisper_msgs.action import STT


class WhisperDemoNode(Node):

    def __init__(self) -> None:
        super().__init__("whisper_demo_node")

        self._action_client = ActionClient(self, STT, "/whisper/listen")

    def listen(self) -> None:

        goal = STT.Goal()

        self._action_client.wait_for_server()
        send_goal_future = self._action_client.send_goal_async(goal)

        rclpy.spin_until_future_complete(self, send_goal_future)
        get_result_future = send_goal_future.result().get_result_async()
        self.get_logger().info("SPEAK")

        rclpy.spin_until_future_complete(self, get_result_future)
        result: STT.Result = get_result_future.result().result
        self.get_logger().info(f"I hear: {result.transcription.text}")
        self.get_logger().info(f"Audio time: {result.transcription.audio_time}")
        self.get_logger().info(
            f"Transcription time: {result.transcription.transcription_time}"
        )


def main():

    rclpy.init()
    node = WhisperDemoNode()
    node.listen()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
