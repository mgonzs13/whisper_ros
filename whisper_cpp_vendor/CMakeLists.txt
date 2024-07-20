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
  GIT_TAG        v1.6.2
)

FetchContent_MakeAvailable(whisper)

set_target_properties(
  whisper PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES $<INSTALL_INTERFACE:.>
  CXX_STANDARD 11
)

install(
  FILES ${whisper_SOURCE_DIR}/ggml.h
  DESTINATION include
)

ament_export_targets(export_whisper HAS_LIBRARY_TARGET)

install(
  TARGETS whisper
  EXPORT export_whisper
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_package()