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


import torch
import numpy as np
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool

from audio_common.utils import msg_to_array
from audio_common_msgs.msg import AudioStamped


class SileroVadNode(Node):

    def __init__(self) -> None:

        super().__init__("silero_vad_node")

        self.recording = False
        self.data: List[float] = []

        self.declare_parameter("enabled", True)
        self.enabled = self.chunk = self.get_parameter(
            "enabled").get_parameter_value().bool_value

        self.init_silero()

        self._enable_srv = self.create_service(
            SetBool, "enable_vad", self.enable_cb)

        self._pub = self.create_publisher(Float32MultiArray, "vad", 10)
        self._sub = self.create_subscription(
            AudioStamped, "audio", self.audio_cb, qos_profile_sensor_data)

        self.get_logger().info("Silero VAD node started")

    @staticmethod
    def normalize(sound):
        abs_max = np.abs(sound).max()
        if abs_max > 0:
            sound *= 1 / abs_max
        sound = sound.squeeze()
        return sound

    def audio_cb(self, msg: AudioStamped) -> None:

        if not self.enabled:
            return

        audio = msg_to_array(msg.audio)
        if audio is None:
            self.get_logger().error(f"Format {msg.audio.info.format} unknown")
            return
        speech_dict = self.vad_iterator(torch.from_numpy(audio))

        if speech_dict:
            self.get_logger().info(str(speech_dict))

            if not self.recording and "start" in speech_dict:
                self.recording = True
                self.data = [0.0] * msg.audio.info.chunk

            elif self.recording and "end" in speech_dict:
                self.recording = False
                vad_msg = Float32MultiArray()
                vad_msg.data = self.data
                self._pub.publish(vad_msg)

        if self.recording:
            self.data.extend((audio).tolist())

    def init_silero(self) -> None:
        model, utils = torch.hub.load(
            repo_or_dir="snakers4/silero-vad",
            model="silero_vad",
            force_reload=False,
            onnx=True
        )
        (_, _, _, VADIterator, _) = utils
        self.vad_iterator = VADIterator(model)

    def enable_cb(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        res.success = True
        self.enabled = req.data

        if self.enabled:
            res.message = "Silero enabled"
        else:
            res.message = "Silero disabled"
            self.recording = False
            self.data = []

        self.get_logger().info(res.message)
        return res


def main():
    rclpy.init()
    node = SileroVadNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
