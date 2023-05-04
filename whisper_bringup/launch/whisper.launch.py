
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    return LaunchDescription([
        Node(
            package="whisper_ros",
            executable="whisper_node",
            name="whisper_node",
            parameters=[{
                "model": LaunchConfiguration("model", default=os.path.abspath(os.path.normpath(os.path.expanduser("~/whisper_models/ggml-base-q5_0.bin")))),
                "language": LaunchConfiguration("language", default="en"),

                "n_threads": LaunchConfiguration("n_threads", default=8),
                "n_max_text_ctx": LaunchConfiguration("n_max_text_ctx", default=16384),
                "offset_ms": LaunchConfiguration("offset_ms", default=0),
                "duration_ms": LaunchConfiguration("duration_ms", default=0),

                "translate": LaunchConfiguration("translate", default=False),
                "no_context": LaunchConfiguration("no_context", default=True),
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
                "max_tokens": LaunchConfiguration("max_tokens", default=32),

                "speed_up": LaunchConfiguration("speed_up", default=False),
                "audio_ctx": LaunchConfiguration("audio_ctx", default=0),

                "suppress_blank": LaunchConfiguration("suppress_blank", default=True),
                "suppress_non_speech_tokens": LaunchConfiguration("suppress_non_speech_tokens", default=False),
            }]
        ),

        Node(
            package="whisper_ros",
            executable="silero_vad_node",
            name="silero_vad_node",
            parameters=[{
                "channels": LaunchConfiguration("channels", default=1),
                "rate": LaunchConfiguration("rate", default=16000),
                "chunk": LaunchConfiguration("chunk", default=4096),
            }]
        )
    ])
