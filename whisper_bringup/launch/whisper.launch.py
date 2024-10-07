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


from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from huggingface_hub import hf_hub_download
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    def run_whisper(context: LaunchContext, repo, file, model_path):
        repo = str(context.perform_substitution(repo))
        file = str(context.perform_substitution(file))
        model_path = str(context.perform_substitution(model_path))

        if not model_path:
            model_path = hf_hub_download(
                repo_id=repo, filename=file, force_download=False)

        params = {
            "sampling_strategy": LaunchConfiguration("sampling_strategy", default="beam_search"),
            "model": LaunchConfiguration("model", default=model_path),
            "openvino_encode_device": LaunchConfiguration("openvino_encode_device", default="CPU"),

            "n_threads": LaunchConfiguration("n_threads", default=4),
            "n_max_text_ctx": LaunchConfiguration("n_max_text_ctx", default=16384),
            "offset_ms": LaunchConfiguration("offset_ms", default=0),
            "duration_ms": LaunchConfiguration("duration_ms", default=0),

            "translate": LaunchConfiguration("translate", default=False),
            "no_context": LaunchConfiguration("no_context", default=True),
            "no_timestamps": LaunchConfiguration("no_timestamps", default=False),
            "single_segment": LaunchConfiguration("single_segment", default=True),
            "print_special": LaunchConfiguration("print_special", default=False),
            "print_progress": LaunchConfiguration("print_progress", default=False),
            "print_realtime": LaunchConfiguration("print_realtime", default=False),
            "print_timestamps": LaunchConfiguration("print_timestamps", default=False),

            "token_timestamps": LaunchConfiguration("token_timestamps", default=False),
            "thold_pt": LaunchConfiguration("thold_pt", default=0.01),
            "thold_ptsum": LaunchConfiguration("thold_ptsum", default=0.01),
            "max_len": LaunchConfiguration("max_len", default=0),
            "split_on_word": LaunchConfiguration("split_on_word", default=False),
            "max_tokens": LaunchConfiguration("max_tokens", default=0),

            "debug_mode": LaunchConfiguration("debug_mode", default=False),
            "audio_ctx": LaunchConfiguration("audio_ctx", default=0),
            "tinydiarize": LaunchConfiguration("tinydiarize", default=False),
            "suppress_regex": LaunchConfiguration("suppress_regex", default=""),

            "language": LaunchConfiguration("language", default="en"),
            "detect_language": LaunchConfiguration("detect_language", default=False),

            "suppress_blank": LaunchConfiguration("suppress_blank", default=True),
            "suppress_non_speech_tokens": LaunchConfiguration("suppress_non_speech_tokens", default=False),

            "temperature": LaunchConfiguration("temperature", default=0.00),
            "max_initial_ts": LaunchConfiguration("max_initial_ts", default=1.00),
            "length_penalty": LaunchConfiguration("length_penalty", default=-1.00),

            "temperature_inc": LaunchConfiguration("temperature_inc", default=0.40),
            "entropy_thold": LaunchConfiguration("entropy_thold", default=2.40),
            "logprob_thold": LaunchConfiguration("logprob_thold", default=-1.00),
            "no_speech_thold": LaunchConfiguration("no_speech_thold", default=0.60),

            "greedy_best_of": LaunchConfiguration("greedy_best_of", default=5),
            "beam_search_beam_size": LaunchConfiguration("beam_search_beam_size", default=5),
            "beam_search_patience": LaunchConfiguration("beam_search_patience", default=-1.00),

            "n_processors": LaunchConfiguration("n_processors", default=1),
            "use_gpu": LaunchConfiguration("use_gpu", default=True),
            "gpu_device": LaunchConfiguration("gpu_device", default=0),
            "flash_attn": LaunchConfiguration("flash_attn", default=False),
            "dtw_n_top": LaunchConfiguration("dtw_n_top", default=-1),
            "dtw_token_timestamps": LaunchConfiguration("dtw_token_timestamps", default=False),
            "dtw_aheads": LaunchConfiguration("dtw_aheads", default="none"),
        }

        return Node(
            package="whisper_ros",
            executable="whisper_server_node",
            name="whisper_node",
            namespace="whisper",
            parameters=[params],
            condition=UnlessCondition(PythonExpression(
                [LaunchConfiguration("stream")]))
        ), Node(
            package="whisper_ros",
            executable="whisper_node",
            name="whisper_node",
            namespace="whisper",
            parameters=[params],
            condition=IfCondition(PythonExpression(
                [LaunchConfiguration("stream")]))
        ),

    stream_cmd = DeclareLaunchArgument(
        "stream",
        default_value="False",
        description="Whether to launch stream or server node")

    model_repo = LaunchConfiguration("model_repo")
    model_repo_cmd = DeclareLaunchArgument(
        "model_repo",
        default_value="ggerganov/whisper.cpp",
        description="Hugging Face model repo")

    model_filename = LaunchConfiguration("model_filename")
    model_filename_cmd = DeclareLaunchArgument(
        "model_filename",
        default_value="ggml-large-v3-turbo-q5_0.bin",
        description="Hugging Face model filename")

    model_path = LaunchConfiguration("model_path")
    model_path_cmd = DeclareLaunchArgument(
        "model_path",
        default_value="",
        description="Local path to the model file")

    return LaunchDescription([
        stream_cmd,
        model_repo_cmd,
        model_filename_cmd,
        model_path_cmd,
        OpaqueFunction(function=run_whisper, args=[
                       model_repo, model_filename, model_path]),

        Node(
            package="whisper_ros",
            executable="silero_vad_node",
            name="silero_vad_node",
            namespace="whisper",
            parameters=[{
                "enabled": LaunchConfiguration("vad_enabled",
                                               default=PythonExpression([LaunchConfiguration("stream")])),
                "threshold": LaunchConfiguration("vad_threshold", default=0.5),
            }],
            remappings=[("audio", "/audio/in")]
        ),

        Node(
            package="audio_common",
            executable="audio_capturer_node",
            name="capturer_node",
            namespace="audio",
            parameters=[{
                "format": LaunchConfiguration("format", default=1),
                "channels": LaunchConfiguration("channels", default=1),
                "rate": LaunchConfiguration("rate", default=16000),
                "chunk": LaunchConfiguration("chunk", default=512),
            }],
            remappings=[("audio", "in")],
            condition=IfCondition(PythonExpression(
                [LaunchConfiguration("launch_audio_capturer", default=True)]))
        )
    ])
