cmake_minimum_required(VERSION 3.6)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

project(tensorrtbuffer)

set(CMAKE_CXX_STANDARD 17)

if(POLICY CMP0146)
    cmake_policy(SET CMP0146 OLD)
endif()

find_package(CUDA REQUIRED)

# Paths are resolved once by the parent project.
if(NOT TENSORRT_INCLUDE_DIRS)
    message(FATAL_ERROR "Configure the parent AirSLAM project with TensorRT_ROOT")
endif()

add_library(tensorrtbuffer SHARED src/logger.cpp)

target_include_directories(tensorrtbuffer
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${CUDA_INCLUDE_DIRS}
    ${TENSORRT_INCLUDE_DIRS}
)

target_link_libraries(tensorrtbuffer
  PRIVATE
    ${CUDA_LIBRARIES}
)
