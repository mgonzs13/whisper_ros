# whisper_ros

This repository provides a set of ROS 2 packages to integrate [whisper.cpp](https://github.com/ggerganov/whisper.cpp) into ROS 2 using [audio_common](https://github.com/mgonzs13/audio_common) [4.0.7](https://github.com/mgonzs13/audio_common/releases/tag/4.0.7). Besides, [silero-vad](https://github.com/snakers4/silero-vad) is used to perform VAD (Voice Activity Detection).

<div align="center">

[![License: MIT](https://img.shields.io/badge/GitHub-MIT-informational)](https://opensource.org/license/mit) [![GitHub release](https://img.shields.io/github/release/mgonzs13/whisper_ros.svg)](https://github.com/mgonzs13/whisper_ros/releases) [![Code Size](https://img.shields.io/github/languages/code-size/mgonzs13/whisper_ros.svg?branch=main)](https://github.com/mgonzs13/whisper_ros?branch=main) [![Last Commit](https://img.shields.io/github/last-commit/mgonzs13/whisper_ros.svg)](https://github.com/mgonzs13/whisper_ros/commits/main) [![GitHub issues](https://img.shields.io/github/issues/mgonzs13/whisper_ros)](https://github.com/mgonzs13/whisper_ros/issues) [![GitHub pull requests](https://img.shields.io/github/issues-pr/mgonzs13/whisper_ros)](https://github.com/mgonzs13/whisper_ros/pulls) [![Contributors](https://img.shields.io/github/contributors/mgonzs13/whisper_ros.svg)](https://github.com/mgonzs13/whisper_ros/graphs/contributors) [![Python Formatter Check](https://github.com/mgonzs13/whisper_ros/actions/workflows/python-formatter.yml/badge.svg?branch=main)](https://github.com/mgonzs13/whisper_ros/actions/workflows/python-formatter.yml?branch=main) [![C++ Formatter Check](https://github.com/mgonzs13/whisper_ros/actions/workflows/cpp-formatter.yml/badge.svg?branch=main)](https://github.com/mgonzs13/whisper_ros/actions/workflows/cpp-formatter.yml?branch=main)

| ROS 2 Distro |                           Branch                            |                                                                                                         Build status                                                                                                         |                                                                 Docker Image                                                                 | Documentation                                                                                                                                                      |
| :----------: | :---------------------------------------------------------: | :--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------: | :------------------------------------------------------------------------------------------------------------------------------------------: | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
|  **Humble**  | [`main`](https://github.com/mgonzs13/whisper_ros/tree/main) |  [![Humble Build](https://github.com/mgonzs13/whisper_ros/actions/workflows/humble-docker-build.yml/badge.svg?branch=main)](https://github.com/mgonzs13/whisper_ros/actions/workflows/humble-docker-build.yml?branch=main)   |  [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-humble-blue)](https://hub.docker.com/r/mgons/whisper_ros/tags?name=humble)  | [![Doxygen Deployment](https://github.com/mgonzs13/whisper_ros/actions/workflows/doxygen-deployment.yml/badge.svg)](https://mgonzs13.github.io/whisper_ros/latest) |
|   **Iron**   | [`main`](https://github.com/mgonzs13/whisper_ros/tree/main) |     [![Iron Build](https://github.com/mgonzs13/whisper_ros/actions/workflows/iron-docker-build.yml/badge.svg?branch=main)](https://github.com/mgonzs13/whisper_ros/actions/workflows/iron-docker-build.yml?branch=main)      |    [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-iron-blue)](https://hub.docker.com/r/mgons/whisper_ros/tags?name=iron)    | [![Doxygen Deployment](https://github.com/mgonzs13/whisper_ros/actions/workflows/doxygen-deployment.yml/badge.svg)](https://mgonzs13.github.io/whisper_ros/latest) |
|  **Jazzy**   | [`main`](https://github.com/mgonzs13/whisper_ros/tree/main) |    [![Jazzy Build](https://github.com/mgonzs13/whisper_ros/actions/workflows/jazzy-docker-build.yml/badge.svg?branch=main)](https://github.com/mgonzs13/whisper_ros/actions/workflows/jazzy-docker-build.yml?branch=main)    |   [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-jazzy-blue)](https://hub.docker.com/r/mgons/whisper_ros/tags?name=jazzy)   | [![Doxygen Deployment](https://github.com/mgonzs13/whisper_ros/actions/workflows/doxygen-deployment.yml/badge.svg)](https://mgonzs13.github.io/whisper_ros/latest) |
|  **Kilted**  | [`main`](https://github.com/mgonzs13/whisper_ros/tree/main) |  [![Kilted Build](https://github.com/mgonzs13/whisper_ros/actions/workflows/kilted-docker-build.yml/badge.svg?branch=main)](https://github.com/mgonzs13/whisper_ros/actions/workflows/kilted-docker-build.yml?branch=main)   |  [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-kilted-blue)](https://hub.docker.com/r/mgons/whisper_ros/tags?name=kilted)  | [![Doxygen Deployment](https://github.com/mgonzs13/whisper_ros/actions/workflows/doxygen-deployment.yml/badge.svg)](https://mgonzs13.github.io/whisper_ros/latest) |
| **Rolling**  | [`main`](https://github.com/mgonzs13/whisper_ros/tree/main) | [![Rolling Build](https://github.com/mgonzs13/whisper_ros/actions/workflows/rolling-docker-build.yml/badge.svg?branch=main)](https://github.com/mgonzs13/whisper_ros/actions/workflows/rolling-docker-build.yml?branch=main) | [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-rolling-blue)](https://hub.docker.com/r/mgons/whisper_ros/tags?name=rolling) | [![Doxygen Deployment](https://github.com/mgonzs13/whisper_ros/actions/workflows/doxygen-deployment.yml/badge.svg)](https://mgonzs13.github.io/whisper_ros/latest) |

</div>

## Table of Contents

1. [Related Projects](#related-projects)
2. [Installation](#installation)
3. [Docker](#docker)
4. [Usage](#usage)
5. [Params](#params)
6. [Demos](#demos)

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

## Params

### Model Parameters

| Param                    | Type     | Default | Description                                                                                |
| ------------------------ | -------- | ------- | ------------------------------------------------------------------------------------------ |
| `model_repo`             | `string` | `""`    | HuggingFace repository for model download.                                                 |
| `model_filename`         | `string` | `""`    | Filename of the model in the repository.                                                   |
| `model_path`             | `string` | `""`    | Local path to the Whisper model file. If empty, the model is downloaded from `model_repo`. |
| `openvino_encode_device` | `string` | `"CPU"` | OpenVINO device for encoder inference (e.g., `"CPU"`, `"GPU"`).                            |

### Sampling Parameters

| Param                   | Type     | Default         | Description                                                                                         |
| ----------------------- | -------- | --------------- | --------------------------------------------------------------------------------------------------- |
| `sampling_strategy`     | `string` | `"beam_search"` | Decoding strategy: `"greedy"` or `"beam_search"`.                                                   |
| `greedy_best_of`        | `int32`  | `-1`            | Number of best candidates to keep when using greedy sampling (`-1` = use whisper.cpp default of 5). |
| `beam_search_beam_size` | `int32`  | `-1`            | Beam size for beam search (`-1` = use whisper.cpp default of 5).                                    |
| `beam_search_patience`  | `float`  | `-1.0`          | Beam search patience factor (`-1.0` = use whisper.cpp default).                                     |

### Transcription Parameters

| Param                  | Type     | Default | Description                                                                           |
| ---------------------- | -------- | ------- | ------------------------------------------------------------------------------------- |
| `n_threads`            | `int32`  | `4`     | Number of threads for processing. Use `-1` to auto-detect from hardware concurrency.  |
| `n_max_text_ctx`       | `int32`  | `16384` | Maximum tokens to use from past text as prompt for the decoder.                       |
| `n_processors`         | `int32`  | `1`     | Number of processors for parallel transcription via `whisper_full_parallel`.          |
| `offset_ms`            | `int32`  | `0`     | Start offset in milliseconds.                                                         |
| `duration_ms`          | `int32`  | `0`     | Audio duration to process in milliseconds (`0` = process all).                        |
| `language`             | `string` | `"en"`  | Spoken language code (e.g., `"en"`, `"es"`, `"de"`). Use `"auto"` for auto-detection. |
| `detect_language`      | `bool`   | `false` | If `true`, auto-detect the spoken language and exit without transcription.            |
| `translate`            | `bool`   | `false` | If `true`, translate the transcription to English.                                    |
| `no_context`           | `bool`   | `true`  | If `true`, do not use past transcription as initial prompt for the decoder.           |
| `no_timestamps`        | `bool`   | `false` | If `true`, do not generate timestamps.                                                |
| `single_segment`       | `bool`   | `false` | If `true`, force single segment output (useful for streaming).                        |
| `initial_prompt`       | `string` | `""`    | Initial prompt text prepended to the decoder context to guide transcription.          |
| `carry_initial_prompt` | `bool`   | `false` | If `true`, always prepend the initial prompt to every decode window.                  |
| `suppress_regex`       | `string` | `""`    | A regular expression that matches tokens to suppress (empty = disabled).              |
| `suppress_blank`       | `bool`   | `true`  | Suppress blank outputs at the beginning of the sampling.                              |
| `suppress_nst`         | `bool`   | `false` | Suppress non-speech tokens.                                                           |
| `audio_ctx`            | `int32`  | `0`     | Overwrite the audio context size (`0` = use model default).                           |

### Token Timestamps Parameters

| Param              | Type    | Default | Description                                                               |
| ------------------ | ------- | ------- | ------------------------------------------------------------------------- |
| `token_timestamps` | `bool`  | `false` | Enable token-level timestamps.                                            |
| `thold_pt`         | `float` | `0.01`  | Timestamp token probability threshold.                                    |
| `thold_ptsum`      | `float` | `0.01`  | Timestamp token sum probability threshold.                                |
| `max_len`          | `int32` | `0`     | Maximum segment length in characters (`0` = no limit).                    |
| `split_on_word`    | `bool`  | `false` | If `true`, split on word rather than on token (when used with `max_len`). |
| `max_tokens`       | `int32` | `0`     | Maximum tokens per segment (`0` = no limit).                              |

### Decoding / Fallback Parameters

| Param             | Type    | Default | Description                                                                                                                                  |
| ----------------- | ------- | ------- | -------------------------------------------------------------------------------------------------------------------------------------------- |
| `temperature`     | `float` | `0.0`   | Initial decoding temperature. Use `0.0` for deterministic output.                                                                            |
| `max_initial_ts`  | `float` | `1.0`   | Maximum initial timestamp allowed.                                                                                                           |
| `length_penalty`  | `float` | `-1.0`  | Length penalty factor (`-1.0` = use whisper.cpp default).                                                                                    |
| `temperature_inc` | `float` | `0.2`   | Temperature increment on decoding fallback.                                                                                                  |
| `entropy_thold`   | `float` | `2.4`   | Entropy threshold for decoding fallback (similar to compression ratio).                                                                      |
| `logprob_thold`   | `float` | `-1.0`  | Log probability threshold for decoding fallback.                                                                                             |
| `no_speech_thold` | `float` | `0.6`   | No-speech probability threshold. If the no-speech probability exceeds this value and `logprob_thold` fails, consider the segment as silence. |

### Speaker Diarization Parameters

| Param         | Type   | Default | Description                                |
| ------------- | ------ | ------- | ------------------------------------------ |
| `tdrz_enable` | `bool` | `false` | Enable tinydiarize speaker turn detection. |

### GPU / Backend Parameters

| Param        | Type    | Default | Description               |
| ------------ | ------- | ------- | ------------------------- |
| `use_gpu`    | `bool`  | `true`  | Enable GPU inference.     |
| `flash_attn` | `bool`  | `true`  | Enable flash attention.   |
| `gpu_device` | `int32` | `0`     | CUDA device index to use. |

### DTW Token Timestamps Parameters

| Param                  | Type     | Default  | Description                                                                                                                                                                                                    |
| ---------------------- | -------- | -------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `dtw_token_timestamps` | `bool`   | `false`  | Enable experimental token-level timestamps with DTW.                                                                                                                                                           |
| `dtw_n_top`            | `int32`  | `-1`     | Number of top text layers for DTW alignment heads (`-1` = disabled).                                                                                                                                           |
| `dtw_aheads`           | `string` | `"none"` | DTW alignment heads preset. Options: `"none"`, `"tiny"`, `"tiny.en"`, `"base"`, `"base.en"`, `"small"`, `"small.en"`, `"medium"`, `"medium.en"`, `"large.v1"`, `"large.v2"`, `"large.v3"`, `"large.v3.turbo"`. |

## Demos

Send a goal action to listen:

```shell
ros2 action send_goal /whisper/listen whisper_msgs/action/STT "{}"
```

Or try the example of a whisper client:

```shell
ros2 run whisper_demos whisper_demo_node
```
