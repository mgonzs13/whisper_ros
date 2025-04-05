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
  GIT_TAG        v1.7.5
)

FetchContent_MakeAvailable(whisper)

# ggml
set_target_properties(
  ggml PROPERTIES
  OUTPUT_NAME "whisper_ggml"
  INTERFACE_INCLUDE_DIRECTORIES "$<BUILD_INTERFACE:${whisper_SOURCE_DIR}/ggml/include>"
  CXX_STANDARD 17
)

set_target_properties(
  ggml-base PROPERTIES
  OUTPUT_NAME "whisper_ggml_base"
  INTERFACE_INCLUDE_DIRECTORIES "$<BUILD_INTERFACE:${whisper_SOURCE_DIR}/ggml/include>"
  CXX_STANDARD 17
)

set_target_properties(
  ggml-cpu PROPERTIES
  OUTPUT_NAME "whisper_ggml_cpu"
  INTERFACE_INCLUDE_DIRECTORIES "$<BUILD_INTERFACE:${whisper_SOURCE_DIR}/ggml/include>"
  CXX_STANDARD 17
)

if(GGML_CUDA)
  set_target_properties(
    ggml-cuda PROPERTIES
    OUTPUT_NAME "whisper_ggml_cuda"
    INTERFACE_INCLUDE_DIRECTORIES "$<BUILD_INTERFACE:${whisper_SOURCE_DIR}/ggml/include>"
    CXX_STANDARD 17
  )
endif()

# whisper
set_target_properties(
  whisper PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES $<INSTALL_INTERFACE:.>
  CXX_STANDARD 17
)

# grammar
add_library(grammar
  ${whisper_SOURCE_DIR}/examples/grammar-parser.cpp
)

target_include_directories(grammar
  PUBLIC
    $<BUILD_INTERFACE:${whisper_SOURCE_DIR}/examples;${whisper_SOURCE_DIR}/include;${whisper_SOURCE_DIR}/ggml/include>
    $<INSTALL_INTERFACE:include>
)

# export
file(GLOB COMMON_HEADERS 
  ${whisper_SOURCE_DIR}/ggml/include/*.
  ${whisper_SOURCE_DIR}/include/*.h
  ${whisper_SOURCE_DIR}/examples/*.h
)
install(
  FILES ${COMMON_HEADERS}
  DESTINATION include
)

set(INSTALL_TARGETS 
  ggml
  ggml-base
  ggml-cpu
  whisper
  grammar
)

if(GGML_CUDA)
  list(APPEND INSTALL_TARGETS ggml-cuda)
endif()

install(
  TARGETS ${INSTALL_TARGETS}
  EXPORT export_whisper
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_whisper HAS_LIBRARY_TARGET)
ament_package()
