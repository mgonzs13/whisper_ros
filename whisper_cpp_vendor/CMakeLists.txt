cmake_minimum_required(VERSION 3.8)
project(whisper_cpp_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)
find_package(ament_cmake REQUIRED)

FetchContent_Declare(
  whisper
  GIT_REPOSITORY https://github.com/ggerganov/whisper.cpp.git
  GIT_TAG        f68298ce06ca3edd6e6f3f21c3d0bb5f073942c3
)

FetchContent_MakeAvailable(whisper)

# Define the target properties
set_target_properties(
  ggml whisper PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "$<BUILD_INTERFACE:${whisper_SOURCE_DIR}/ggml/include>"
  CXX_STANDARD 11
)

ament_export_targets(export_whisper HAS_LIBRARY_TARGET)

install(
  FILES ${whisper_SOURCE_DIR}/ggml/include/ggml.h
  DESTINATION include
)

install(
  TARGETS ggml whisper
  EXPORT export_whisper
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_package()
