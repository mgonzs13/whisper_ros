# whisper_ros

This repository provides a set of ROS 2 packages to integrate [whisper.cpp](https://github.com/ggerganov/whisper.cpp) into ROS 2 using [audio_common](https://github.com/mgonzs13/audio_common) [4.0.0](https://github.com/mgonzs13/audio_common/releases/tag/4.0.0). Besides, [silero-vad](https://github.com/snakers4/silero-vad) is used to perform VAD (Voice Activity Detection).

## Table of Contents

- [whisper\_ros](#whisper_ros)
  - [Table of Contents](#table-of-contents)
  - [Related Projects](#related-projects)
  - [Installation](#installation)
  - [Docker](#docker)
  - [Usage](#usage)
  - [Demos](#demos)

## Related Projects

- [chatbot_ros](https://github.com/mgonzs13/chatbot_ros) &rarr; This chatbot, integrated into ROS 2, uses whisper_ros, to listen to people speech; and [llama_ros](https://github.com/mgonzs13/llama_ros/tree/main), to generate responses. The chatbot is controlled by a state machine created with [YASMIN](https://github.com/uleroboticsgroup/yasmin).

## Installation

To run whisper_ros with CUDA, first, you must install the [CUDA Toolkit](https://developer.nvidia.com/cuda-toolkit).

```shell
$ cd ~/ros2_ws/src
$ git clone https://github.com/mgonzs13/audio_common.git
$ git clone https://github.com/mgonzs13/whisper_ros.git
$ pip3 install -r whisper_ros/requirements.txt
$ cd ~/ros2_ws
$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build --cmake-args -DGGML_CUDA=ON # add this for CUDA
```

## Docker

Build the whisper_ros docker. Additionally, you can choose to build whisper_ros with CUDA (`USE_CUDA`) and choose the CUDA version (`CUDA_VERSION`). Remember that you have to use `DOCKER_BUILDKIT=0` to compile whisper_ros with CUDA when building the image.

```shell
$ DOCKER_BUILDKIT=0 docker build -t whisper_ros --build-arg USE_CUDA=1 --build-arg CUDA_VERSION=12-6 .
```

Run the docker container. If you want to use CUDA, you have to install the [NVIDIA Container Tollkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) and add `--gpus all`.

```shell
$ docker run -it --rm --gpus all whisper_ros
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
