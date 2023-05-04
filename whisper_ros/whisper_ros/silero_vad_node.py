#!/usr/bin/env python3

from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import pyaudio
import torch
import numpy as np


class SileroVadNode(Node):

    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 16000
    CHUNK = 4096

    def __init__(self) -> None:

        super().__init__("siler_vad_node")

        # params
        self.declare_parameters(
            namespace="",
            parameters=[
                ("channels", 1),
                ("rate", 16000),
                ("chunk", 4096)
            ]
        )

        self.CHANNELS = self.get_parameter(
            "channels").get_parameter_value().integer_value
        self.RATE = self.get_parameter(
            "rate").get_parameter_value().integer_value
        self.CHUNK = self.get_parameter(
            "chunk").get_parameter_value().integer_value

        # recording
        self.recording = False
        self.data: List[float] = [0.0] * self.CHUNK

        # pyaudio
        audio = pyaudio.PyAudio()
        self.stream = audio.open(
            format=self.FORMAT,
            channels=self.CHANNELS,
            rate=self.RATE,
            input=True,
            frames_per_buffer=self.CHUNK
        )

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
        self.create_timer(0.001, self.work)

        self.get_logger().info("Silero VAD node started")

    @staticmethod
    def normalize(sound):
        abs_max = np.abs(sound).max()
        if abs_max > 0:
            sound *= 1 / abs_max
        sound = sound.squeeze()
        return sound

    def work(self) -> None:
        data = self.stream.read(self.CHUNK)
        audio = np.frombuffer(
            data, np.int16).flatten().astype(np.float32) / 32768.0
        speech_dict = self.vad_iterator(torch.from_numpy(audio))

        if speech_dict:
            self.get_logger().info(str(speech_dict))

            if not self.recording and "start" in speech_dict:
                self.recording = True

            elif self.recording and "end" in speech_dict:
                self.recording = False

                msg = Float32MultiArray()
                msg.data = self.data
                self.pub_.publish(msg)

                self.data = [0.0] * self.CHUNK

        if self.recording:
            self.data.extend((audio).tolist())

    def destroy_node(self):
        self.stream.stop_stream()
        self.stream.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = SileroVadNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
