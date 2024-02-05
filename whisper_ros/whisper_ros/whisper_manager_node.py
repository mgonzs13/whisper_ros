#!/usr/bin/env python3

# MIT License

# Copyright (c) 2023  Miguel Ángel González Santamarta

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


import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool
from whisper_msgs.action import STT


class WhisperManagerNode(Node):

    def __init__(self) -> None:

        super().__init__("whisper_manager_node")

        self.whisper_text_lock = threading.Lock()
        self.vad_lock = threading.Lock()
        self.whisper_text = ""
        self.vad_data = None

        self._enable_client = self.create_client(
            SetBool, "enable_vad",
            callback_group=ReentrantCallbackGroup())
        self._text_sub = self.create_subscription(
            String, "text", self.whisper_cb, 10,
            callback_group=ReentrantCallbackGroup())
        self._vad_sub = self.create_subscription(
            Float32MultiArray, "vad", self.vad_cb, 10,
            callback_group=ReentrantCallbackGroup())

        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._action_server = ActionServer(
            self, STT, "listen",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

        self.get_logger().info("Whisper manager node started")

    def enable_silero(self, enable: bool) -> None:
        req = SetBool.Request()
        req.data = enable
        self._enable_client.call(req)

    def whisper_cb(self, msg: String) -> None:

        with self.whisper_text_lock:
            self.whisper_text = msg.data

            if not self.whisper_text:
                with self.vad_lock:
                    if self.vad_data is not None:
                        self.enable_silero(True)
                    self.vad_data = None

    def vad_cb(self, msg: Float32MultiArray) -> None:
        with self.vad_lock:
            if self.vad_data is None:
                self.enable_silero(False)
            self.vad_data = msg

    def destroy_node(self) -> bool:
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request: ServerGoalHandle) -> int:
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle) -> None:
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self._goal_handle.abort()
                self.enable_silero(False)
            self._goal_handle = goal_handle
        goal_handle.execute()

    def cancel_callback(self, goal_handle: ServerGoalHandle) -> None:
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle) -> STT.Result:

        result = STT.Result()
        no_text = True

        # reset data and enable silero
        with self.whisper_text_lock:
            self.whisper_text = ""
        with self.vad_lock:
            self.vad_data = None
        self.enable_silero(True)

        # wait for whisper text
        self.get_logger().info("Waiting for whisper text")
        while no_text:
            time.sleep(0.01)

            if not goal_handle.is_active:
                return result

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.enable_silero(False)
                return result

            with self.whisper_text_lock:
                if self.whisper_text:
                    no_text = False
                    result.text = self.whisper_text

        goal_handle.succeed()
        self.enable_silero(False)
        return result


def main():
    rclpy.init()
    node = WhisperManagerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
