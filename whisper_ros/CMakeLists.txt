cmake_minimum_required(VERSION 3.8)
project(whisper_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_package(std_msgs REQUIRED)

INCLUDE_DIRECTORIES(
  include
  whisper_cpp/examples
)

add_subdirectory(whisper_cpp)

add_executable(whisper_node src/whisper_node.cpp src/whisper.cpp)
target_link_libraries(whisper_node whisper)
ament_target_dependencies(whisper_node rclcpp audio_common_msgs std_msgs)

# INSTALL
install(TARGETS
  whisper_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
