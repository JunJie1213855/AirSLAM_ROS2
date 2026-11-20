cmake_minimum_required(VERSION 3.6)
project(tensorrtbuffer)

set(CMAKE_CXX_STANDARD 17)

if(POLICY CMP0146)
    cmake_policy(SET CMP0146 OLD)
endif()

find_package(CUDA REQUIRED)

# TensorRT — inherited from parent scope (AirSLAM/CMakeLists.txt)
if(NOT DEFINED TENSORRT_INCLUDE_DIRS)
    set(TENSORRT_INCLUDE_DIRS $ENV{HOME}/lib/Deploy/TensorRT-10.9.0.34/include)
endif()
if(NOT DEFINED TENSORRT_LIB_DIRS)
    set(TENSORRT_LIB_DIRS $ENV{HOME}/lib/Deploy/TensorRT-10.9.0.34/lib)
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
