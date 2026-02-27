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

    config_dir = os.path.join(get_package_share_directory("whisper_bringup"), "config")

    stream_cmd = DeclareLaunchArgument(
        "stream",
        default_value="False",
        description="Whether to launch stream or server node",
    )

    whisper_params_file = LaunchConfiguration(
        "whisper_params_file",
        default=os.path.join(config_dir, "whisper.yaml"),
    )

    silero_vad_params_file = LaunchConfiguration(
        "silero_vad_params_file",
        default=os.path.join(config_dir, "silero_vad.yaml"),
    )

    audio_capturer_params_file = LaunchConfiguration(
        "audio_capturer_params_file",
        default=os.path.join(config_dir, "audio_capturer.yaml"),
    )

    return LaunchDescription(
        [
            stream_cmd,
            Node(
                package="whisper_ros",
                executable="whisper_server_node",
                name="whisper_node",
                namespace="whisper",
                parameters=[whisper_params_file],
                condition=UnlessCondition(
                    PythonExpression([LaunchConfiguration("stream")])
                ),
            ),
            Node(
                package="whisper_ros",
                executable="whisper_node",
                name="whisper_node",
                namespace="whisper",
                parameters=[whisper_params_file],
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
                    "silero_vad_params_file": silero_vad_params_file,
                }.items(),
            ),
            Node(
                package="audio_common",
                executable="audio_capturer_node",
                name="capturer_node",
                namespace="audio",
                parameters=[audio_capturer_params_file],
                remappings=[("audio", "in")],
                condition=IfCondition(
                    PythonExpression(
                        [LaunchConfiguration("launch_audio_capturer", default=True)]
                    )
                ),
            ),
        ]
    )
