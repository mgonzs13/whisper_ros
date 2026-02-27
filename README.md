# whisper_ros

This repository provides a set of ROS 2 packages to integrate [whisper.cpp](https://github.com/ggerganov/whisper.cpp) into ROS 2 using [audio_common](https://github.com/mgonzs13/audio_common). Besides, [silero-vad](https://github.com/snakers4/silero-vad) is used to perform VAD (Voice Activity Detection).

<div align="center">

[![License: MIT](https://img.shields.io/badge/GitHub-MIT-informational)](https://opensource.org/license/mit) [![GitHub release](https://img.shields.io/github/release/mgonzs13/whisper_ros.svg)](https://github.com/mgonzs13/whisper_ros/releases) [![Code Size](https://img.shields.io/github/languages/code-size/mgonzs13/whisper_ros.svg?branch=main)](https://github.com/mgonzs13/whisper_ros?branch=main) [![Last Commit](https://img.shields.io/github/last-commit/mgonzs13/whisper_ros.svg)](https://github.com/mgonzs13/whisper_ros/commits/main) [![GitHub issues](https://img.shields.io/github/issues/mgonzs13/whisper_ros)](https://github.com/mgonzs13/whisper_ros/issues) [![GitHub pull requests](https://img.shields.io/github/issues-pr/mgonzs13/whisper_ros)](https://github.com/mgonzs13/whisper_ros/pulls) [![Contributors](https://img.shields.io/github/contributors/mgonzs13/whisper_ros.svg)](https://github.com/mgonzs13/whisper_ros/graphs/contributors) [![Python Formatter Check](https://github.com/mgonzs13/whisper_ros/actions/workflows/python-formatter.yml/badge.svg?branch=main)](https://github.com/mgonzs13/whisper_ros/actions/workflows/python-formatter.yml?branch=main) [![C++ Formatter Check](https://github.com/mgonzs13/whisper_ros/actions/workflows/cpp-formatter.yml/badge.svg?branch=main)](https://github.com/mgonzs13/whisper_ros/actions/workflows/cpp-formatter.yml?branch=main) [![Doxygen Deployment](https://github.com/mgonzs13/whisper_ros/actions/workflows/doxygen-deployment.yml/badge.svg)](https://mgonzs13.github.io/whisper_ros/latest)

| ROS 2 Distro |                           Branch                            |                                                                                                       Build status                                                                                                       |                                                                 Docker Image                                                                 |
| :----------: | :---------------------------------------------------------: | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------: | :------------------------------------------------------------------------------------------------------------------------------------------: |
|  **Humble**  | [`main`](https://github.com/mgonzs13/whisper_ros/tree/main) |  [![Humble Build](https://github.com/mgonzs13/whisper_ros/actions/workflows/humble-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/whisper_ros/actions/workflows/humble-build-test.yml?branch=main)   |  [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-humble-blue)](https://hub.docker.com/r/mgons/whisper_ros/tags?name=humble)  |
|   **Iron**   | [`main`](https://github.com/mgonzs13/whisper_ros/tree/main) |     [![Iron Build](https://github.com/mgonzs13/whisper_ros/actions/workflows/iron-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/whisper_ros/actions/workflows/iron-build-test.yml?branch=main)      |    [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-iron-blue)](https://hub.docker.com/r/mgons/whisper_ros/tags?name=iron)    |
|  **Jazzy**   | [`main`](https://github.com/mgonzs13/whisper_ros/tree/main) |    [![Jazzy Build](https://github.com/mgonzs13/whisper_ros/actions/workflows/jazzy-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/whisper_ros/actions/workflows/jazzy-build-test.yml?branch=main)    |   [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-jazzy-blue)](https://hub.docker.com/r/mgons/whisper_ros/tags?name=jazzy)   |
|  **Kilted**  | [`main`](https://github.com/mgonzs13/whisper_ros/tree/main) |  [![Kilted Build](https://github.com/mgonzs13/whisper_ros/actions/workflows/kilted-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/whisper_ros/actions/workflows/kilted-build-test.yml?branch=main)   |  [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-kilted-blue)](https://hub.docker.com/r/mgons/whisper_ros/tags?name=kilted)  |
| **Rolling**  | [`main`](https://github.com/mgonzs13/whisper_ros/tree/main) | [![Rolling Build](https://github.com/mgonzs13/whisper_ros/actions/workflows/rolling-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/whisper_ros/actions/workflows/rolling-build-test.yml?branch=main) | [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-rolling-blue)](https://hub.docker.com/r/mgons/whisper_ros/tags?name=rolling) |

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
git clone https://github.com/mgonzs13/whisper_ros.git
cd ~/ros2_ws
vcs import src < src/whisper_ros/dependencies.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DGGML_CUDA=ON -DONNX_GPU=ON # To use CUDA on Whisper and on Silero, respectively
```

## Docker

Build the whisper_ros docker. Additionally, you can choose to build whisper_ros with CUDA (`USE_CUDA`) and choose the CUDA version (`CUDA_VERSION`). Remember that you have to use `DOCKER_BUILDKIT=0` to compile whisper_ros with CUDA when building the image.

```shell
DOCKER_BUILDKIT=0 docker build -t whisper_ros --build-arg USE_CUDA=1 --build-arg CUDA_VERSION=12-6 .
```

Run the docker container. If you want to use CUDA, you have to install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) and add `--gpus all`.

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

### Model Parameters (`model.*`)

| Param                          | Type     | Default | Description                                                                                |
| ------------------------------ | -------- | ------- | ------------------------------------------------------------------------------------------ |
| `model.repo`                   | `string` | `""`    | HuggingFace repository for model download.                                                 |
| `model.filename`               | `string` | `""`    | Filename of the model in the repository.                                                   |
| `model.path`                   | `string` | `""`    | Local path to the Whisper model file. If empty, the model is downloaded from `model.repo`. |
| `model.openvino_encode_device` | `string` | `"CPU"` | OpenVINO device for encoder inference (e.g., `"CPU"`, `"GPU"`).                            |

### Sampling Parameters (`sampling.*`)

| Param                            | Type     | Default         | Description                                                     |
| -------------------------------- | -------- | --------------- | --------------------------------------------------------------- |
| `sampling.strategy`              | `string` | `"beam_search"` | Decoding strategy: `"greedy"` or `"beam_search"`.               |
| `sampling.greedy_best_of`        | `int32`  | `5`             | Number of best candidates to keep when using greedy sampling.   |
| `sampling.beam_search_beam_size` | `int32`  | `5`             | Beam size for beam search.                                      |
| `sampling.beam_search_patience`  | `float`  | `-1.0`          | Beam search patience factor (`-1.0` = use whisper.cpp default). |

### Transcription Parameters (`transcription.*`)

| Param                                | Type     | Default | Description                                                                           |
| ------------------------------------ | -------- | ------- | ------------------------------------------------------------------------------------- |
| `transcription.n_threads`            | `int32`  | `4`     | Number of threads for processing. Use `-1` to auto-detect from hardware concurrency.  |
| `transcription.n_max_text_ctx`       | `int32`  | `16384` | Maximum tokens to use from past text as prompt for the decoder.                       |
| `transcription.n_processors`         | `int32`  | `1`     | Number of processors for parallel transcription via `whisper_full_parallel`.          |
| `transcription.offset_ms`            | `int32`  | `0`     | Start offset in milliseconds.                                                         |
| `transcription.duration_ms`          | `int32`  | `0`     | Audio duration to process in milliseconds (`0` = process all).                        |
| `transcription.audio_ctx`            | `int32`  | `0`     | Overwrite the audio context size (`0` = use model default).                           |
| `transcription.language`             | `string` | `"en"`  | Spoken language code (e.g., `"en"`, `"es"`, `"de"`). Use `"auto"` for auto-detection. |
| `transcription.detect_language`      | `bool`   | `false` | If `true`, auto-detect the spoken language and exit without transcription.            |
| `transcription.translate`            | `bool`   | `false` | If `true`, translate the transcription to English.                                    |
| `transcription.no_context`           | `bool`   | `true`  | If `true`, do not use past transcription as initial prompt for the decoder.           |
| `transcription.no_timestamps`        | `bool`   | `false` | If `true`, do not generate timestamps.                                                |
| `transcription.single_segment`       | `bool`   | `false` | If `true`, force single segment output (useful for streaming).                        |
| `transcription.initial_prompt`       | `string` | `""`    | Initial prompt text prepended to the decoder context to guide transcription.          |
| `transcription.carry_initial_prompt` | `bool`   | `false` | If `true`, always prepend the initial prompt to every decode window.                  |
| `transcription.suppress_regex`       | `string` | `""`    | A regular expression that matches tokens to suppress (empty = disabled).              |
| `transcription.suppress_blank`       | `bool`   | `true`  | Suppress blank outputs at the beginning of the sampling.                              |
| `transcription.suppress_nst`         | `bool`   | `false` | Suppress non-speech tokens.                                                           |

### Token Timestamps Parameters (`token_timestamps.*`)

| Param                            | Type    | Default | Description                                                               |
| -------------------------------- | ------- | ------- | ------------------------------------------------------------------------- |
| `token_timestamps.enabled`       | `bool`  | `false` | Enable token-level timestamps.                                            |
| `token_timestamps.thold_pt`      | `float` | `0.01`  | Timestamp token probability threshold.                                    |
| `token_timestamps.thold_ptsum`   | `float` | `0.01`  | Timestamp token sum probability threshold.                                |
| `token_timestamps.max_len`       | `int32` | `0`     | Maximum segment length in characters (`0` = no limit).                    |
| `token_timestamps.split_on_word` | `bool`  | `false` | If `true`, split on word rather than on token (when used with `max_len`). |
| `token_timestamps.max_tokens`    | `int32` | `0`     | Maximum tokens per segment (`0` = no limit).                              |

### Decoding / Fallback Parameters (`decoding.*`)

| Param                      | Type    | Default | Description                                                                                                                                  |
| -------------------------- | ------- | ------- | -------------------------------------------------------------------------------------------------------------------------------------------- |
| `decoding.temperature`     | `float` | `0.0`   | Initial decoding temperature. Use `0.0` for deterministic output.                                                                            |
| `decoding.max_initial_ts`  | `float` | `1.0`   | Maximum initial timestamp allowed.                                                                                                           |
| `decoding.length_penalty`  | `float` | `-1.0`  | Length penalty factor (`-1.0` = use whisper.cpp default).                                                                                    |
| `decoding.temperature_inc` | `float` | `0.2`   | Temperature increment on decoding fallback.                                                                                                  |
| `decoding.entropy_thold`   | `float` | `2.4`   | Entropy threshold for decoding fallback (similar to compression ratio).                                                                      |
| `decoding.logprob_thold`   | `float` | `-1.0`  | Log probability threshold for decoding fallback.                                                                                             |
| `decoding.no_speech_thold` | `float` | `0.6`   | No-speech probability threshold. If the no-speech probability exceeds this value and `logprob_thold` fails, consider the segment as silence. |

### Speaker Diarization Parameters (`diarization.*`)

| Param                     | Type   | Default | Description                                |
| ------------------------- | ------ | ------- | ------------------------------------------ |
| `diarization.tdrz_enable` | `bool` | `false` | Enable tinydiarize speaker turn detection. |

### GPU / Backend Parameters (`gpu.*`)

| Param            | Type    | Default | Description               |
| ---------------- | ------- | ------- | ------------------------- |
| `gpu.enabled`    | `bool`  | `true`  | Enable GPU inference.     |
| `gpu.flash_attn` | `bool`  | `true`  | Enable flash attention.   |
| `gpu.device`     | `int32` | `0`     | CUDA device index to use. |

### DTW Token Timestamps Parameters (`dtw.*`)

| Param                  | Type     | Default  | Description                                                                                                                                                                                                    |
| ---------------------- | -------- | -------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `dtw.token_timestamps` | `bool`   | `false`  | Enable experimental token-level timestamps with DTW.                                                                                                                                                           |
| `dtw.n_top`            | `int32`  | `-1`     | Number of top text layers for DTW alignment heads (`-1` = disabled).                                                                                                                                           |
| `dtw.aheads`           | `string` | `"none"` | DTW alignment heads preset. Options: `"none"`, `"tiny"`, `"tiny.en"`, `"base"`, `"base.en"`, `"small"`, `"small.en"`, `"medium"`, `"medium.en"`, `"large.v1"`, `"large.v2"`, `"large.v3"`, `"large.v3.turbo"`. |

## Demos

Send a goal action to listen:

```shell
ros2 action send_goal /whisper/listen whisper_msgs/action/STT "{}"
```

Or try the example of a whisper client:

```shell
ros2 run whisper_demos whisper_demo_node
```
