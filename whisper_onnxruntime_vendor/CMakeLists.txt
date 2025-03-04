cmake_minimum_required(VERSION 3.8)
project(whisper_onnxruntime_vendor)

option(ONNX_GPU "Use GPU for inference" OFF)

set(GPU "")

# Determine the architecture
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(ARCHITECTURE "x64")
    if(ONNX_GPU)
        set(GPU "-gpu")
    endif()
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(ARCHITECTURE "aarch64")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# Set variables for the package
set(ONNXRUNTIME_VERSION "1.20.0")  # Specify the desired ONNX Runtime version
set(ONNXRUNTIME_URL "https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-${ARCHITECTURE}${GPU}-${ONNXRUNTIME_VERSION}.tgz")

# ROS 2 package configuration
find_package(ament_cmake REQUIRED)

# Define a vendor package installation directory
set(ONNXRUNTIME_INSTALL_DIR "${CMAKE_BINARY_DIR}/onnxruntime-linux-${ARCHITECTURE}${GPU}-${ONNXRUNTIME_VERSION}")

# Add a custom target to download and extract the prebuilt ONNX Runtime
find_program(CURL_EXECUTABLE curl REQUIRED)

if(NOT CURL_EXECUTABLE)
    message(FATAL_ERROR "curl is required to download ONNX Runtime but was not found.")
endif()

# Add custom command to download and extract the ONNX Runtime
add_custom_target(download_onnxruntime ALL
    COMMENT "Downloading and extracting ONNX Runtime ${ONNXRUNTIME_VERSION}"
    COMMAND ${CURL_EXECUTABLE} -L -o onnxruntime.tgz ${ONNXRUNTIME_URL} >/dev/null 2>&1
    COMMAND ${CMAKE_COMMAND} -E tar xzf onnxruntime.tgz
)

# Install the ONNX Runtime library and include files
install(DIRECTORY ${ONNXRUNTIME_INSTALL_DIR}/lib DESTINATION .)
install(DIRECTORY ${ONNXRUNTIME_INSTALL_DIR}/include DESTINATION .)

# Export the onnxruntime library for downstream packages
ament_export_include_directories(include)
ament_export_libraries(onnxruntime)
ament_package()
