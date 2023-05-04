# whisper_ros

This repositiory provides a set of ROS 2 packages to integrate [whisper.cpp](https://github.com/ggerganov/whisper.cpp) into ROS 2. Besides, [seliro-vad](https://github.com/snakers4/silero-vad) is used to perform VAD (Voice Activity Detection).

## Installation

```shell
$ cd ~/ros2_ws/src
$ git clone --recurse-submodules https://github.com/AGI4ROS/whisper_ros.git
$ pip3 install -r whisper_ros/requirements.txt
$ cd ~/ros2_ws
$ colcon build
```

## Usage

```shell
$ ros2 launch whisper_bringup whisper.launch.py
```
