cmake_minimum_required(VERSION 3.8)
project(whisper_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_package(whisper_msgs REQUIRED)
find_package(whisper_cpp_vendor REQUIRED)
find_package(whisper_onnxruntime_vendor REQUIRED)
find_package(whisper_hfhub_vendor REQUIRED)
find_library(PORTAUDIO_LIB portaudio REQUIRED)

include_directories(
  include
  ${PORTAUDIO_INCLUDE_DIR}
)

# whisper_node
add_executable(whisper_node
  src/whisper_main.cpp
  src/whisper_ros/whisper_node.cpp
  src/whisper_ros/whisper_base_node.cpp
  src/whisper_ros/whisper.cpp
  src/whisper_utils/logs.cpp
  src/whisper_utils/download_model.cpp
)
target_link_libraries(whisper_node 
  whisper_cpp_vendor::grammar
  whisper_cpp_vendor::whisper
)
ament_target_dependencies(whisper_node
  rclcpp
  rclcpp_lifecycle
  std_msgs
  std_srvs
  whisper_msgs
  whisper_cpp_vendor
  whisper_hfhub_vendor
)

# whisper_server_node
add_executable(whisper_server_node
  src/whisper_server_main.cpp
  src/whisper_ros/whisper_server_node.cpp
  src/whisper_ros/whisper_base_node.cpp
  src/whisper_ros/whisper.cpp
  src/whisper_utils/logs.cpp
  src/whisper_utils/download_model.cpp
)
target_link_libraries(whisper_server_node
  whisper_cpp_vendor::grammar
  whisper_cpp_vendor::whisper
)
ament_target_dependencies(whisper_server_node
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  std_srvs
  whisper_msgs
  whisper_cpp_vendor
  whisper_hfhub_vendor
)

# silero_vad_node
add_executable(silero_vad_node
  src/silero_vad_main.cpp
  src/silero_vad/silero_vad_node.cpp
  src/silero_vad/vad_iterator.cpp
  src/silero_vad/timestamp.cpp
  src/whisper_utils/logs.cpp
  src/whisper_utils/download_model.cpp
)
target_link_libraries(silero_vad_node ${PORTAUDIO_LIB})
ament_target_dependencies(silero_vad_node
  rclcpp
  rclcpp_lifecycle
  std_msgs
  std_srvs
  audio_common_msgs
  whisper_onnxruntime_vendor
  whisper_hfhub_vendor
)

# Export dependencies
ament_export_dependencies(whisper_cpp_vendor)
ament_export_dependencies(whisper_onnxruntime_vendor)

# Install
install(TARGETS
  whisper_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  whisper_server_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  silero_vad_node
  DESTINATION lib/${PROJECT_NAME}
)

# TEST
if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format(CONFIG_FILE .clang-format)
endif()

ament_package()
