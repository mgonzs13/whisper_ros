cmake_minimum_required(VERSION 3.8)
project(whisper_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# CUDA
if(DEFINED ENV{WHISPER_CUDA})
    message(STATUS "WHISPER_CUDA")
    string(TOLOWER "$ENV{WHISPER_CUDA}" WHISPER_CUDA)
    if(WHISPER_CUDA STREQUAL "on")
        message(STATUS "Compiling whisper_ros with CUDA")
        set(GGML_CUDA ON)
    endif()
endif()

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
  whisper_ros/whisper_demo_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME whisper_demo_node
)

ament_package()
