ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO} AS deps

SHELL ["/bin/bash", "-c"]

# Copy sources and import dependencies via vcs
WORKDIR /root/ros2_ws
COPY . src/whisper_ros/
RUN apt-get update && \
    apt-get install -y python3-vcstool && \
    vcs import src < src/whisper_ros/dependencies.repos && \
    rm -rf /var/lib/apt/lists/*

# Install ROS dependencies
RUN apt-get update && \
    rosdep update --include-eol-distros && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Install CUDA toolkit (optional)
ARG USE_CUDA=0
ARG CUDA_VERSION=12-6

RUN if [ "$USE_CUDA" = "1" ]; then \
    wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i cuda-keyring_1.1-1_all.deb && \
    rm cuda-keyring_1.1-1_all.deb && \
    apt-get update && \
    apt-get install -y cuda-toolkit-${CUDA_VERSION} && \
    rm -rf /var/lib/apt/lists/*; \
    fi

# Build the workspace with colcon
FROM deps AS builder
ARG CMAKE_BUILD_TYPE=Release
ARG USE_CUDA=0

ENV PATH=/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    if [ "$USE_CUDA" = "1" ]; then \
        colcon build --cmake-args -DGGML_CUDA=ON -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}; \
    else \
        colcon build --cmake-args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}; \
    fi

# Source the workspace on login
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
