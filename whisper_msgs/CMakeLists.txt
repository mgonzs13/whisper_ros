cmake_minimum_required(VERSION 3.8)
project(whisper_msgs)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/STT.action"
  "msg/GrammarConfig.msg"
  "srv/SetGrammar.srv"
  "srv/SetInitPrompt.srv"
)

ament_package()
