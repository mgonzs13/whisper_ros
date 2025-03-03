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
from launch import LaunchDescription, LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from huggingface_hub import hf_hub_download


def generate_launch_description():

    def run_silero_vad(context: LaunchContext, repo, file, model_path, use_cuda):
        repo = str(context.perform_substitution(repo))
        file = str(context.perform_substitution(file))
        model_path = str(context.perform_substitution(model_path))

        if not model_path:
            model_path = hf_hub_download(
                repo_id=repo, filename=file, force_download=False
            )

        return (
            Node(
                package="whisper_ros",
                executable="silero_vad_node",
                name="silero_vad_node",
                namespace="whisper",
                parameters=[
                    {
                        "enabled": LaunchConfiguration("enabled", default=True),
                        "model_path": model_path,
                        "sample_rate": LaunchConfiguration("sample_rate", default=16000),
                        "frame_size_ms": LaunchConfiguration("frame_size_ms", default=32),
                        "threshold": LaunchConfiguration("threshold", default=0.5),
                        "min_silence_ms": LaunchConfiguration(
                            "min_silence_ms", default=128
                        ),
                        "speech_pad_ms": LaunchConfiguration("speech_pad_ms", default=32),
                        "use_cuda": use_cuda,
                    }
                ],
                remappings=[("audio", "/audio/in")],
            ),
        )

    model_repo = LaunchConfiguration("model_repo")
    model_repo_cmd = DeclareLaunchArgument(
        "model_repo",
        default_value="mgonzs13/silero-vad-onnx",
        description="Hugging Face model repo",
    )

    model_filename = LaunchConfiguration("model_filename")
    model_filename_cmd = DeclareLaunchArgument(
        "model_filename",
        default_value="silero_vad.onnx",
        description="Hugging Face model filename",
    )

    model_path = LaunchConfiguration("model_path")
    model_path_cmd = DeclareLaunchArgument(
        "model_path",
        default_value="",
        description="Local path to the model file",
    )
    
    use_cuda = LaunchConfiguration("use_cuda")
    use_cuda_cmd = DeclareLaunchArgument(
        "use_cuda",
        default_value="False",
        description="Use CUDA for inference",
    )
    

    return LaunchDescription(
        [
            model_repo_cmd,
            model_filename_cmd,
            model_path_cmd,
            use_cuda_cmd,
            OpaqueFunction(
                function=run_silero_vad,
                args=[model_repo, model_filename, model_path, use_cuda],
            ),
        ]
    )
