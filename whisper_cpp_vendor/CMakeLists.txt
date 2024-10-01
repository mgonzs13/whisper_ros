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
  GIT_TAG        2ef717b293fe93872cc3a03ca77942936a281959
)

FetchContent_MakeAvailable(whisper)

# ggml
set_target_properties(
  ggml PROPERTIES
  OUTPUT_NAME "whisper_ggml"
  INTERFACE_INCLUDE_DIRECTORIES "$<BUILD_INTERFACE:${whisper_SOURCE_DIR}/ggml/include>"
  CXX_STANDARD 11
)

# whisper
set_target_properties(
  whisper PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES $<INSTALL_INTERFACE:.>
  CXX_STANDARD 11
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
file(GLOB COMMON_HEADERS ${whisper_SOURCE_DIR}/include/*.h ${whisper_SOURCE_DIR}/examples/*.h)
install(
  FILES ${COMMON_HEADERS}
  DESTINATION include
)

install(
  TARGETS ggml whisper grammar
  EXPORT export_whisper
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_whisper HAS_LIBRARY_TARGET)
ament_package()
