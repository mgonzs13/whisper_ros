#!/usr/bin/env python3

from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray

from audio_common.utils import msg_to_array
from audio_common_msgs.msg import AudioStamped

import torch
import numpy as np


class SileroVadNode(Node):

    def __init__(self) -> None:

        super().__init__("siler_vad_node")

        # recording
        self.recording = False
        self.data: List[float] = []

        # silerio torch model
        model, utils = torch.hub.load(repo_or_dir="snakers4/silero-vad",
                                      model="silero_vad",
                                      force_reload=True,
                                      onnx=False)
        (get_speech_timestamps,
         save_audio,
         read_audio,
         VADIterator,
         collect_chunks) = utils
        self.vad_iterator = VADIterator(model)

        # ros
        self.pub_ = self.create_publisher(Float32MultiArray, "/silero_vad", 10)
        self.sub_ = self.create_subscription(
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

        audio = msg_to_array(msg.audio.audio_data, msg.audio.info.format)
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

                msg = Float32MultiArray()
                msg.data = self.data
                self.pub_.publish(msg)

        if self.recording:
            self.data.extend((audio).tolist())


def main():
    rclpy.init()
    node = SileroVadNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
