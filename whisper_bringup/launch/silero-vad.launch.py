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


from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="whisper_ros",
                executable="silero_vad_node",
                name="silero_vad_node",
                namespace="whisper",
                parameters=[
                    {
                        "enabled": LaunchConfiguration("enabled", default=True),
                        "model_repo": LaunchConfiguration(
                            "model_repo", default="mgonzs13/silero-vad-onnx"
                        ),
                        "model_filename": LaunchConfiguration(
                            "model_filename", default="silero_vad.onnx"
                        ),
                        "model_path": LaunchConfiguration("model_path", default=""),
                        "sample_rate": LaunchConfiguration("sample_rate", default=16000),
                        "frame_size_ms": LaunchConfiguration("frame_size_ms", default=32),
                        "threshold": LaunchConfiguration("threshold", default=0.5),
                        "min_silence_ms": LaunchConfiguration(
                            "min_silence_ms", default=128
                        ),
                        "speech_pad_ms": LaunchConfiguration("speech_pad_ms", default=32),
                        "use_cuda": LaunchConfiguration("use_cuda", default=False),
                    }
                ],
                remappings=[("audio", "/audio/in")],
            ),
        ]
    )
