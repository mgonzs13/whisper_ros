# whisper_ros

This repositiory provides a set of ROS 2 packages to integrate [whisper.cpp](https://github.com/ggerganov/whisper.cpp) into ROS 2. Besides, [silero-vad](https://github.com/snakers4/silero-vad) is used to perform VAD (Voice Activity Detection).

## Installation

```shell
$ cd ~/ros2_ws/src
$ git clone https://github.com/mgonzs13/audio_common.git
$ git clone --recurse-submodules https://github.com/AGI4ROS/whisper_ros.git
$ sudo apt install portaudio19-dev
$ pip3 install -r audio_common/requirements.txt
$ pip3 install -r whisper_ros/requirements.txt
$ cd ~/ros2_ws
$ colcon build
```

## Usage

Download the models and place them in `~/whisper_models`.

```shell
$ ros2 launch whisper_bringup whisper.launch.py
```

```shell
$ ros2 action send_goal /whisper/listen whisper_msgs/action/STT "{}"
```
