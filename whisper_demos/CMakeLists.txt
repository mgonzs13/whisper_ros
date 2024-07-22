cmake_minimum_required(VERSION 3.8)
project(whisper_demos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(PROGRAMS
  whisper_demos/whisper_demo_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME whisper_demo_node
)

ament_package()
