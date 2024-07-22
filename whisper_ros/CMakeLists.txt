cmake_minimum_required(VERSION 3.8)
project(whisper_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(whisper_msgs REQUIRED)
find_package(whisper_cpp_vendor REQUIRED)

include_directories(include)

add_executable(whisper_node
  src/whisper_main.cpp
  src/whisper_node.cpp
  src/whisper.cpp
)
target_link_libraries(whisper_node whisper_cpp_vendor::grammar whisper_cpp_vendor::whisper)
ament_target_dependencies(whisper_node rclcpp std_msgs std_srvs whisper_msgs whisper_cpp_vendor)
ament_export_dependencies(whisper_cpp_vendor)

# INSTALL
install(TARGETS
  whisper_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  whisper_ros/silero_vad_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME silero_vad_node
)

install(PROGRAMS
  whisper_ros/whisper_manager_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME whisper_manager_node
)

ament_package()
