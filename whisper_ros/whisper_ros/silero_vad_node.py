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


import numpy as np
from typing import List
from silero_vad import VADIterator, load_silero_vad

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool

from audio_common.utils import msg_to_array
from audio_common_msgs.msg import AudioStamped


def int2float(audio_data: np.ndarray) -> np.ndarray:

    if audio_data is None or len(audio_data) == 0:
        return None

    from_type = type(audio_data[0])

    if from_type == np.uint8:
        audio_data = (audio_data.astype(np.float32) - 128) / 128.0

    elif from_type == np.int8:
        audio_data = audio_data.astype(np.float32) / 128.0

    elif from_type == np.int16:
        audio_data = audio_data.astype(np.float32) / 32768.0

    elif from_type == np.int32:
        audio_data = audio_data.astype(np.float32) / 2147483648.0

    elif from_type == np.float32:
        audio_data = audio_data

    else:
        return None

    return audio_data


class SileroVadNode(Node):

    def __init__(self) -> None:

        super().__init__("silero_vad_node")

        self.recording = False
        self.data: List[float] = []

        self.declare_parameter("enabled", True)
        self.enabled = self.chunk = self.get_parameter(
            "enabled").get_parameter_value().bool_value

        self.declare_parameter("threshold", 0.5)
        self.threshold = self.chunk = self.get_parameter(
            "threshold").get_parameter_value().double_value

        # create silero model
        model = load_silero_vad(onnx=True)
        self.vad_iterator = VADIterator(model, threshold=self.threshold)

        # srvs, subs, pubs
        self._enable_srv = self.create_service(
            SetBool, "enable_vad", self.enable_cb)
        self._pub = self.create_publisher(Float32MultiArray, "vad", 10)
        self._sub = self.create_subscription(
            AudioStamped, "audio", self.audio_cb, qos_profile_sensor_data)

        self.get_logger().info("Silero VAD node started")

    def audio_cb(self, msg: AudioStamped) -> None:

        if not self.enabled:
            return

        audio_array = int2float(msg_to_array(msg.audio))

        if audio_array is None:
            self.get_logger().error(f"Format {msg.audio.info.format} unknown")
            return
        speech_dict = self.vad_iterator(audio_array)

        if speech_dict:
            self.get_logger().info(str(speech_dict))

            if not self.recording and "start" in speech_dict:
                self.recording = True
                self.data = []

            elif self.recording and "end" in speech_dict:
                self.recording = False
                self.data.extend(audio_array.tolist())

                if len(self.data) / msg.audio.info.rate < 1.0:
                    pad_size = msg.audio.info.chunk + \
                        msg.audio.info.rate - len(self.data)
                    self.data.extend(pad_size * [0.0])

                vad_msg = Float32MultiArray()
                vad_msg.data = self.data
                self._pub.publish(vad_msg)

        if self.recording:
            self.data.extend(audio_array.tolist())

    def enable_cb(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        res.success = True
        self.enabled = req.data

        if self.enabled:
            res.message = "Silero enabled"
            self.vad_iterator.reset_states()
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
