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


import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    stream_cmd = DeclareLaunchArgument(
        "stream",
        default_value="False",
        description="Whether to launch stream or server node",
    )

    whisper_params = {
        "sampling_strategy": LaunchConfiguration(
            "sampling_strategy", default="beam_search"
        ),
        "model_repo": LaunchConfiguration("model_repo", default="ggerganov/whisper.cpp"),
        "model_filename": LaunchConfiguration(
            "model_filename", default="ggml-large-v3-turbo-q5_0.bin"
        ),
        "model_path": LaunchConfiguration("model_path", default=""),
        "openvino_encode_device": LaunchConfiguration(
            "openvino_encode_device", default="CPU"
        ),
        "n_threads": LaunchConfiguration("n_threads", default=4),
        "n_max_text_ctx": LaunchConfiguration("n_max_text_ctx", default=16384),
        "offset_ms": LaunchConfiguration("offset_ms", default=0),
        "duration_ms": LaunchConfiguration("duration_ms", default=0),
        "translate": LaunchConfiguration("translate", default=False),
        "no_context": LaunchConfiguration("no_context", default=True),
        "single_segment": LaunchConfiguration("single_segment", default=True),
        "token_timestamps": LaunchConfiguration("token_timestamps", default=False),
        "thold_pt": LaunchConfiguration("thold_pt", default=0.01),
        "thold_ptsum": LaunchConfiguration("thold_ptsum", default=0.01),
        "max_len": LaunchConfiguration("max_len", default=0),
        "split_on_word": LaunchConfiguration("split_on_word", default=False),
        "max_tokens": LaunchConfiguration("max_tokens", default=0),
        "audio_ctx": LaunchConfiguration("audio_ctx", default=0),
        "suppress_regex": LaunchConfiguration("suppress_regex", default=""),
        "language": LaunchConfiguration("language", default="en"),
        "detect_language": LaunchConfiguration("detect_language", default=False),
        "suppress_blank": LaunchConfiguration("suppress_blank", default=True),
        "suppress_nst": LaunchConfiguration("suppress_nst", default=False),
        "temperature": LaunchConfiguration("temperature", default=0.00),
        "max_initial_ts": LaunchConfiguration("max_initial_ts", default=1.00),
        "length_penalty": LaunchConfiguration("length_penalty", default=-1.00),
        "temperature_inc": LaunchConfiguration("temperature_inc", default=0.40),
        "entropy_thold": LaunchConfiguration("entropy_thold", default=2.40),
        "logprob_thold": LaunchConfiguration("logprob_thold", default=-1.00),
        "no_speech_thold": LaunchConfiguration("no_speech_thold", default=0.60),
        "greedy_best_of": LaunchConfiguration("greedy_best_of", default=5),
        "beam_search_beam_size": LaunchConfiguration("beam_search_beam_size", default=5),
        "beam_search_patience": LaunchConfiguration(
            "beam_search_patience", default=-1.00
        ),
        "n_processors": LaunchConfiguration("n_processors", default=1),
        "flash_attn": LaunchConfiguration("flash_attn", default=False),
        "use_gpu": LaunchConfiguration("use_gpu", default=True),
        "gpu_device": LaunchConfiguration("gpu_device", default=0),
        "flash_attn": LaunchConfiguration("flash_attn", default=False),
        "dtw_n_top": LaunchConfiguration("dtw_n_top", default=-1),
        "dtw_token_timestamps": LaunchConfiguration(
            "dtw_token_timestamps", default=False
        ),
        "dtw_aheads": LaunchConfiguration("dtw_aheads", default="none"),
    }

    silero_vad_model_repo = LaunchConfiguration("silero_vad_model_repo")
    silero_vad_model_repo_cmd = DeclareLaunchArgument(
        "silero_vad_model_repo",
        default_value="mgonzs13/silero-vad-onnx",
        description="Hugging Face model repo for SileroVAD",
    )

    silero_vad_model_filename = LaunchConfiguration("silero_vad_model_filename")
    silero_vad_model_filename_cmd = DeclareLaunchArgument(
        "silero_vad_model_filename",
        default_value="silero_vad.onnx",
        description="Hugging Face model filename for SileroVAD",
    )

    silero_vad_model_path = LaunchConfiguration("silero_vad_model_path")
    silero_vad_model_path_cmd = DeclareLaunchArgument(
        "silero_vad_model_path",
        default_value="",
        description="Local path to the model file for SileroVAD",
    )

    silero_vad_use_cuda = LaunchConfiguration("silero_vad_use_cuda")
    silero_vad_use_cuda_cmd = DeclareLaunchArgument(
        "silero_vad_use_cuda",
        default_value="False",
        description="Whether to use CUDA for SileroVAD",
    )

    return LaunchDescription(
        [
            stream_cmd,
            silero_vad_model_repo_cmd,
            silero_vad_model_filename_cmd,
            silero_vad_model_path_cmd,
            silero_vad_use_cuda_cmd,
            Node(
                package="whisper_ros",
                executable="whisper_server_node",
                name="whisper_node",
                namespace="whisper",
                parameters=[whisper_params],
                condition=UnlessCondition(
                    PythonExpression([LaunchConfiguration("stream")])
                ),
            ),
            Node(
                package="whisper_ros",
                executable="whisper_node",
                name="whisper_node",
                namespace="whisper",
                parameters=[whisper_params],
                condition=IfCondition(PythonExpression([LaunchConfiguration("stream")])),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("whisper_bringup"),
                        "launch",
                        "silero-vad.launch.py",
                    )
                ),
                launch_arguments={
                    "enabled": LaunchConfiguration(
                        "vad_enabled",
                        default=PythonExpression([LaunchConfiguration("stream")]),
                    ),
                    "model_repo": silero_vad_model_repo,
                    "model_filename": silero_vad_model_filename,
                    "model_path": silero_vad_model_path,
                    "use_cuda": silero_vad_use_cuda,
                }.items(),
            ),
            Node(
                package="audio_common",
                executable="audio_capturer_node",
                name="capturer_node",
                namespace="audio",
                parameters=[
                    {
                        "format": LaunchConfiguration("format", default=1),
                        "channels": LaunchConfiguration("channels", default=1),
                        "rate": LaunchConfiguration("rate", default=16000),
                        "chunk": LaunchConfiguration("chunk", default=512),
                    }
                ],
                remappings=[("audio", "in")],
                condition=IfCondition(
                    PythonExpression(
                        [LaunchConfiguration("launch_audio_capturer", default=True)]
                    )
                ),
            ),
        ]
    )
