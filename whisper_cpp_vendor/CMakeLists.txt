cmake_minimum_required(VERSION 3.10)
project(whisper_cpp_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)
find_package(ament_cmake REQUIRED)

FetchContent_Declare(
  whisper
  GIT_REPOSITORY https://github.com/ggerganov/whisper.cpp.git
  # master (2026-10-19) = whisper.cpp v1.9.5 + ggml 0.27. The Hexagon backend in
  # ggml 0.26 mis-handles this model's HMX matmuls (wrong logits) and rejects
  # the flash-attention KV cache copies, so stay on master until a v1.9.6 tag
  # with ggml 0.27 is released.
  GIT_TAG        231638a0717b7822784297e5980965ffcec05810
  # VENTUNO Q (aarch64): unique Hexagon skel names (whisper_ros and llama_ros
  # share install/lib and ADSP_LIBRARY_PATH) and a lazy OpenCL context init.
  PATCH_COMMAND  ${CMAKE_COMMAND}
                 -DWHISPER_SOURCE_DIR=<SOURCE_DIR>
                 -P ${CMAKE_CURRENT_SOURCE_DIR}/patch-whisper-for-ventuno.cmake
)

set(WHISPER_BUILD_IS_DEV OFF CACHE BOOL "whisper: dev build" FORCE)

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

if(GGML_BLAS)
  set_target_properties(
    ggml-blas PROPERTIES
    OUTPUT_NAME "whisper_ggml_blas"
    INTERFACE_INCLUDE_DIRECTORIES "$<BUILD_INTERFACE:${whisper_SOURCE_DIR}/ggml/include>"
    CXX_STANDARD 17
  )
endif()

if(GGML_OPENCL)
  set_target_properties(
    ggml-opencl PROPERTIES
    OUTPUT_NAME "whisper_ggml_opencl"
    INTERFACE_INCLUDE_DIRECTORIES "$<BUILD_INTERFACE:${whisper_SOURCE_DIR}/ggml/include>"
    CXX_STANDARD 17
  )
endif()

if(GGML_HEXAGON)
  set_target_properties(
    ggml-hexagon PROPERTIES
    OUTPUT_NAME "whisper_ggml_hexagon"
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
  ${whisper_SOURCE_DIR}/ggml/include/*.h
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

if(GGML_BLAS)
  list(APPEND INSTALL_TARGETS ggml-blas)
endif()

if(GGML_OPENCL)
  list(APPEND INSTALL_TARGETS ggml-opencl)
endif()

if(GGML_HEXAGON)
  list(APPEND INSTALL_TARGETS ggml-hexagon)
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
