cmake_minimum_required(VERSION 3.8)
project(whisper_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(WHISPER_CUBLAS "whisper: support for cuBLAS" ON)

# find dependencies
find_package(SDL2 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(whisper_msgs REQUIRED)

INCLUDE_DIRECTORIES(
  include
  whisper_cpp/examples
)

add_subdirectory(whisper_cpp)

add_executable(whisper_node
  src/whisper_main.cpp
  src/whisper_node.cpp
  src/whisper.cpp
  whisper_cpp/examples/common-sdl.cpp
  whisper_cpp/examples/common.cpp
  whisper_cpp/examples/grammar-parser.cpp
)
target_include_directories(whisper_node PUBLIC ${SDL2_INCLUDE_DIRS})
target_link_libraries(whisper_node whisper ${SDL2_LIBRARIES})
ament_target_dependencies(whisper_node rclcpp std_msgs std_srvs whisper_msgs)

# INSTALL
install(TARGETS
  whisper_node
  DESTINATION lib/${PROJECT_NAME})

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

install(PROGRAMS
  whisper_ros/whisper_client_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME whisper_client_node
)

ament_package()
