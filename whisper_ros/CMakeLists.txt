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
find_package(whisper_msgs REQUIRED)
find_package(whisper_cpp_vendor REQUIRED)

include_directories(include)

add_executable(whisper_node
  src/whisper_main.cpp
  src/whisper_ros/whisper_node.cpp
  src/whisper_ros/whisper_base_node.cpp
  src/whisper_ros/whisper.cpp
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
)

add_executable(whisper_server_node
  src/whisper_server_main.cpp
  src/whisper_ros/whisper_server_node.cpp
  src/whisper_ros/whisper_base_node.cpp
  src/whisper_ros/whisper.cpp
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
)

ament_export_dependencies(whisper_cpp_vendor)


# INSTALL
install(TARGETS
  whisper_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  whisper_server_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  whisper_ros/silero_vad_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME silero_vad_node
)

ament_package()
