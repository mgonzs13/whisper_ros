cmake_minimum_required(VERSION 3.8)
project(whisper_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/STT.action"
  DEPENDENCIES
)

ament_package()
