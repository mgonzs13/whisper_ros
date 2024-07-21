# whisper_ros

This repository provides a set of ROS 2 packages to integrate [whisper.cpp](https://github.com/ggerganov/whisper.cpp) into ROS 2 using [audio_common](https://github.com/mgonzs13/audio_common). Besides, [silero-vad](https://github.com/snakers4/silero-vad) is used to perform VAD (Voice Activity Detection).

## Installation

```shell
$ cd ~/ros2_ws/src
$ git clone https://github.com/mgonzs13/audio_common.git
$ git clone https://github.com/mgonzs13/whisper_ros.git
$ sudo apt install portaudio19-dev
$ pip3 install -r audio_common/requirements.txt
$ pip3 install -r whisper_ros/requirements.txt
$ cd ~/ros2_ws
$ colcon build --cmake-args -DGGML_CUDA=ON
```

### CUDA

To run llama_ros with CUDA, first, you must install the [CUDA Toolkit](https://developer.nvidia.com/cuda-toolkit). Then, you have to add `--cmake-args -DGGML_CUDA=ON` to colcon.

## Usage

Run Silero for VAD and Whisper for STT:

```shell
$ ros2 launch whisper_bringup whisper.launch.py
```

Send a goal action to listen:

```shell
$ ros2 action send_goal /whisper/listen whisper_msgs/action/STT "{}"
```

Or try the example of a whisper client:

```shell
$ ros2 run whisper_ros whisper_demo_node
```
