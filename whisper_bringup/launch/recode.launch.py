from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():

    return LaunchDescription([

        Node(
            package="audio_common",
            executable="audio_capturer_node",
            name="capturer_node",
            namespace="audio",
            parameters=[{
                "format": LaunchConfiguration("channels", default=1),
                "channels": LaunchConfiguration("channels", default=1),
                "rate": LaunchConfiguration("rate", default=16000),
                "chunk": LaunchConfiguration("chunk", default=4096),
            }],
            remappings=[("audio", "in")],
            condition=IfCondition(PythonExpression(
                [LaunchConfiguration("launch_audio_capturer", default=True)]))
        )
    ])
