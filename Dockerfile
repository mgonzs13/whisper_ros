ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO} AS deps

# Create ros2_ws and copy files
WORKDIR /root/ros2_ws
SHELL ["/bin/bash", "-c"]
COPY . /root/ros2_ws/src

# Install dependencies
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/mgonzs13/audio_common.git

WORKDIR /root/ros2_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash
RUN apt-get update
RUN rosdep update --include-eol-distros && rosdep install --from-paths src --ignore-src -r -y
RUN rosdep install --from-paths src --ignore-src -r -y

# Install CUDA nvcc
ARG USE_CUDA
ARG CUDA_VERSION=12-6

RUN if [ "$USE_CUDA" = "1" ]; then \
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i cuda-keyring_1.1-1_all.deb && \
    rm cuda-keyring_1.1-1_all.deb; \
    apt-get update && apt-get install -y cuda-toolkit-$CUDA_VERSION; \
    echo "export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}" >> ~/.bashrc; \
    echo "export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" >> ~/.bashrc; \
    fi

# Build the ws with colcon
FROM deps AS builder
ARG CMAKE_BUILD_TYPE=Release

ENV PATH=/usr/local/cuda/bin${PATH:+:${PATH}}
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    if [ "$USE_CUDA" = "1" ]; then \
    source ~/.bashrc && \
    colcon build --cmake-args -DGGML_CUDA=ON; \
    else \
    colcon build; \
    fi

# Source the ROS 2 setup file
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Run a default command, e.g., starting a bash shell
CMD ["bash"]
