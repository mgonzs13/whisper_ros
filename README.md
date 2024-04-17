# whisper_ros

This repository provides a set of ROS 2 packages to integrate [whisper.cpp](https://github.com/ggerganov/whisper.cpp) into ROS 2 using [audio_common](https://github.com/mgonzs13/audio_common). Besides, [silero-vad](https://github.com/snakers4/silero-vad) is used to perform VAD (Voice Activity Detection).

## Installation

```shell
$ cd ~/ros2_ws/src
$ git clone https://github.com/mgonzs13/audio_common.git
$ git clone --recursive git@github.com:SaboProject/Whisper_ros.git
# $ git clone --recurse-submodules https://github.com/mgonzs13/whisper_ros.git
$ sudo apt install portaudio19-dev
$ pip3 install -r audio_common/requirements.txt
$ pip3 install -r whisper_ros/requirements.txt
$ cd ~/ros2_ws
$ colcon build
```

### CUDA

To run llama_ros with CUDA, you have to install the [CUDA Toolkit](https://developer.nvidia.com/cuda-toolkit) and the following line in the [CMakeLists.txt](whisper_ros/CMakeLists.txt) must be uncommented:

```
option(WHISPER_CUBLAS "whisper: support for cuBLAS" ON)
```

## Usage

Run Silero for VAD and Whisper for STT:

```shell
$ ros2 run audio_common audio_capturer_node
$ ros2 launch whisper_bringup whisper.launch.py
```

Send a goal action to listen:

```shell
$ ros2 action send_goal /whisper/listen whisper_msgs/action/STT "{}"
```

Or try the example of a whisper client:

```shell
$ ros2 run whisper_ros whisper_client_node
```
