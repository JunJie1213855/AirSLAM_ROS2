cmake_minimum_required(VERSION 3.5)
project(air_vo)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
add_compile_options(-w)

# TensorRT 10.9.0.34 (same stack as AirSLAM_ROS2).
# Override for local builds: -DTENSORRT_DIR=$HOME/lib/TensorRT-<version>
set(TENSORRT_DIR "$ENV{HOME}/lib/Deploy/TensorRT-10.9.0.34" CACHE PATH "TensorRT root directory")
set(TENSORRT_INCLUDE_DIRS ${TENSORRT_DIR}/include)
set(TENSORRT_LIB_DIRS ${TENSORRT_DIR}/lib)

add_subdirectory(${PROJECT_SOURCE_DIR}/3rdparty/tensorrtbuffer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(OpenCV 4.2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CUDA REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(g2o REQUIRED)
find_package(Gflags REQUIRED)
find_package(Glog REQUIRED)

# CUDA 13.0: find_package does not always set library paths
link_directories(${CUDA_TOOLKIT_ROOT_DIR}/lib64 ${CUDA_TOOLKIT_ROOT_DIR}/lib)

find_library(TENSORRT_LIBRARY_INFER nvinfer HINTS ${TENSORRT_LIB_DIRS} REQUIRED)
find_library(TENSORRT_LIBRARY_ONNX nvonnxparser HINTS ${TENSORRT_LIB_DIRS} REQUIRED)

add_library(${PROJECT_NAME}_lib SHARED
  src/g2o_optimization/vertex_line3d.cc
  src/g2o_optimization/edge_project_line.cc
  src/g2o_optimization/edge_project_stereo_line.cc
  src/g2o_optimization/g2o_optimization.cc
  src/super_point.cpp
  src/super_glue.cpp
  src/utils.cc
  src/camera.cc
  src/dataset.cc
  src/frame.cc
  src/point_matching.cc
  src/mappoint.cc
  src/mapline.cc
  src/line_processor.cc
  src/ros_publisher.cc
  src/map.cc
  src/map_builder.cc
  src/timer.cc
)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${TENSORRT_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${GFLAGS_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}_lib
  PUBLIC
  rclcpp cv_bridge message_filters geometry_msgs nav_msgs
  sensor_msgs std_msgs visualization_msgs
)

target_link_libraries(${PROJECT_NAME}_lib
  PUBLIC
    g2o::core g2o::types_slam3d g2o::types_sba g2o::types_slam3d_addons g2o::solver_eigen
  PRIVATE
    ${TENSORRT_LIBRARY_INFER}
    ${TENSORRT_LIBRARY_ONNX}
    ${CUDA_LIBRARIES}
    cudart
    tensorrtbuffer
  PUBLIC
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Boost_SERIALIZATION_LIBRARY}
    ${GFLAGS_LIBRARIES}
    ${GLOG_LIBRARIES}
    yaml-cpp
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  cv_bridge
  message_filters
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
)

add_executable(air_vo demo/visual_odometry.cpp)
target_link_libraries(air_vo ${PROJECT_NAME}_lib)

add_executable(air_vo_ros demo/visual_odometry_ros.cpp)
target_link_libraries(air_vo_ros ${PROJECT_NAME}_lib)

add_executable(test_feature demo/test_feature.cpp)
target_link_libraries(test_feature ${PROJECT_NAME}_lib)

install(TARGETS ${PROJECT_NAME}_lib tensorrtbuffer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS air_vo air_vo_ros test_feature
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY configs/ DESTINATION share/${PROJECT_NAME}/configs)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY output/ DESTINATION share/${PROJECT_NAME}/output)

ament_package()
