# whisper_ros

This repository provides a set of ROS 2 packages to integrate [whisper.cpp](https://github.com/ggerganov/whisper.cpp) into ROS 2 using [audio_common](https://github.com/mgonzs13/audio_common) [4.0.5](https://github.com/mgonzs13/audio_common/releases/tag/4.0.5). Besides, [silero-vad](https://github.com/snakers4/silero-vad) is used to perform VAD (Voice Activity Detection).

<div align="center">

[![License: MIT](https://img.shields.io/badge/GitHub-MIT-informational)](https://opensource.org/license/mit) [![GitHub release](https://img.shields.io/github/release/mgonzs13/whisper_ros.svg)](https://github.com/mgonzs13/whisper_ros/releases) [![Code Size](https://img.shields.io/github/languages/code-size/mgonzs13/whisper_ros.svg?branch=main)](https://github.com/mgonzs13/whisper_ros?branch=main) [![Last Commit](https://img.shields.io/github/last-commit/mgonzs13/whisper_ros.svg)](https://github.com/mgonzs13/whisper_ros/commits/main) [![GitHub issues](https://img.shields.io/github/issues/mgonzs13/whisper_ros)](https://github.com/mgonzs13/whisper_ros/issues) [![GitHub pull requests](https://img.shields.io/github/issues-pr/mgonzs13/whisper_ros)](https://github.com/mgonzs13/whisper_ros/pulls) [![Contributors](https://img.shields.io/github/contributors/mgonzs13/whisper_ros.svg)](https://github.com/mgonzs13/whisper_ros/graphs/contributors) [![Python Formatter Check](https://github.com/mgonzs13/whisper_ros/actions/workflows/python-formatter.yml/badge.svg?branch=main)](https://github.com/mgonzs13/whisper_ros/actions/workflows/python-formatter.yml?branch=main) [![C++ Formatter Check](https://github.com/mgonzs13/whisper_ros/actions/workflows/cpp-formatter.yml/badge.svg?branch=main)](https://github.com/mgonzs13/whisper_ros/actions/workflows/cpp-formatter.yml?branch=main)

| ROS 2 Distro |                           Branch                            |                                                                                                         Build status                                                                                                         |                                                                 Docker Image                                                                 | Documentation                                                                                                                                                      |
| :----------: | :---------------------------------------------------------: | :--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------: | :------------------------------------------------------------------------------------------------------------------------------------------: | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
|  **Humble**  | [`main`](https://github.com/mgonzs13/whisper_ros/tree/main) |  [![Humble Build](https://github.com/mgonzs13/whisper_ros/actions/workflows/humble-docker-build.yml/badge.svg?branch=main)](https://github.com/mgonzs13/whisper_ros/actions/workflows/humble-docker-build.yml?branch=main)   |  [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-humble-blue)](https://hub.docker.com/r/mgons/whisper_ros/tags?name=humble)  | [![Doxygen Deployment](https://github.com/mgonzs13/whisper_ros/actions/workflows/doxygen-deployment.yml/badge.svg)](https://mgonzs13.github.io/whisper_ros/latest) |
|   **Iron**   | [`main`](https://github.com/mgonzs13/whisper_ros/tree/main) |     [![Iron Build](https://github.com/mgonzs13/whisper_ros/actions/workflows/iron-docker-build.yml/badge.svg?branch=main)](https://github.com/mgonzs13/whisper_ros/actions/workflows/iron-docker-build.yml?branch=main)      |    [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-iron-blue)](https://hub.docker.com/r/mgons/whisper_ros/tags?name=iron)    | [![Doxygen Deployment](https://github.com/mgonzs13/whisper_ros/actions/workflows/doxygen-deployment.yml/badge.svg)](https://mgonzs13.github.io/whisper_ros/latest) |
|  **Jazzy**   | [`main`](https://github.com/mgonzs13/whisper_ros/tree/main) |    [![Jazzy Build](https://github.com/mgonzs13/whisper_ros/actions/workflows/jazzy-docker-build.yml/badge.svg?branch=main)](https://github.com/mgonzs13/whisper_ros/actions/workflows/jazzy-docker-build.yml?branch=main)    |   [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-jazzy-blue)](https://hub.docker.com/r/mgons/whisper_ros/tags?name=jazzy)   | [![Doxygen Deployment](https://github.com/mgonzs13/whisper_ros/actions/workflows/doxygen-deployment.yml/badge.svg)](https://mgonzs13.github.io/whisper_ros/latest) |
| **Rolling**  | [`main`](https://github.com/mgonzs13/whisper_ros/tree/main) | [![Rolling Build](https://github.com/mgonzs13/whisper_ros/actions/workflows/rolling-docker-build.yml/badge.svg?branch=main)](https://github.com/mgonzs13/whisper_ros/actions/workflows/rolling-docker-build.yml?branch=main) | [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-rolling-blue)](https://hub.docker.com/r/mgons/whisper_ros/tags?name=rolling) | [![Doxygen Deployment](https://github.com/mgonzs13/whisper_ros/actions/workflows/doxygen-deployment.yml/badge.svg)](https://mgonzs13.github.io/whisper_ros/latest) |

</div>

## Table of Contents

1. [Related Projects](#related-projects)
2. [Installation](#installation)
3. [Docker](#docker)
4. [Usage](#usage)
5. [Demos](#demos)

## Related Projects

- [chatbot_ros](https://github.com/mgonzs13/chatbot_ros) &rarr; This chatbot, integrated into ROS 2, uses whisper_ros, to listen to people speech; and [llama_ros](https://github.com/mgonzs13/llama_ros/tree/main), to generate responses. The chatbot is controlled by a state machine created with [YASMIN](https://github.com/uleroboticsgroup/yasmin).

## Installation

To run whisper_ros with CUDA, first, you must install the [CUDA Toolkit](https://developer.nvidia.com/cuda-toolkit). To run SileroVAD with ONNX and CUDA, you must install the [cuDNN](https://developer.nvidia.com/cudnn-downloads).

```shell
cd ~/ros2_ws/src
git clone https://github.com/mgonzs13/audio_common.git
git clone https://github.com/mgonzs13/whisper_ros.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DGGML_CUDA=ON -DONNX_GPU=ON # To use CUDA on Whisper and on Silero, respectively
```

## Docker

Build the whisper_ros docker. Additionally, you can choose to build whisper_ros with CUDA (`USE_CUDA`) and choose the CUDA version (`CUDA_VERSION`). Remember that you have to use `DOCKER_BUILDKIT=0` to compile whisper_ros with CUDA when building the image.

```shell
DOCKER_BUILDKIT=0 docker build -t whisper_ros --build-arg USE_CUDA=1 --build-arg CUDA_VERSION=12-6 .
```

Run the docker container. If you want to use CUDA, you have to install the [NVIDIA Container Tollkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) and add `--gpus all`.

```shell
docker run -it --rm --gpus all whisper_ros
```

## Usage

Run Silero for VAD and Whisper for STT:

```shell
ros2 launch whisper_bringup whisper.launch.py
```

Add the parameter `silero_vad_use_cuda:=True` to use Silero with CUDA.

## Demos

Send a goal action to listen:

```shell
ros2 action send_goal /whisper/listen whisper_msgs/action/STT "{}"
```

Or try the example of a whisper client:

```shell
ros2 run whisper_demos whisper_demo_node
```
