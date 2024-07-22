# whisper_ros

This repository provides a set of ROS 2 packages to integrate [whisper.cpp](https://github.com/ggerganov/whisper.cpp) into ROS 2 using [audio_common](https://github.com/mgonzs13/audio_common). Besides, [silero-vad](https://github.com/snakers4/silero-vad) is used to perform VAD (Voice Activity Detection).

## Table of Contents

1. [Related Projects](#related-projects)
2. [Installation](#installation)
3. [Usage](#usage)
4. [Demos](#demos)

## Related Projects

- [chatbot_ros](https://github.com/mgonzs13/chatbot_ros) &rarr; This chatbot, integrated into ROS 2, uses whisper_ros, to listen to people speech; and [llama_ros](https://github.com/mgonzs13/llama_ros/tree/main), to generate responses. The chatbot is controlled by a state machine created with [YASMIN](https://github.com/uleroboticsgroup/yasmin).

## Installation

To run llama_ros with CUDA, first, you must install the [CUDA Toolkit](https://developer.nvidia.com/cuda-toolkit).

```shell
$ cd ~/ros2_ws/src
$ git clone https://github.com/mgonzs13/audio_common.git
$ git clone https://github.com/mgonzs13/whisper_ros.git
$ sudo apt install portaudio19-dev
$ pip3 install -r audio_common/requirements.txt
$ pip3 install -r whisper_ros/requirements.txt
$ cd ~/ros2_ws
$ colcon build --cmake-args -DGGML_CUDA=ON # add this for CUDA
```

## Usage

Run Silero for VAD and Whisper for STT:

```shell
$ ros2 launch whisper_bringup whisper.launch.py
```

## Demos

Send a goal action to listen:

```shell
$ ros2 action send_goal /whisper/listen whisper_msgs/action/STT "{}"
```

Or try the example of a whisper client:

```shell
$ ros2 run whisper_demos whisper_demo_node
```
