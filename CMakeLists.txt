cmake_minimum_required(VERSION 3.5)
project(air_slam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
add_compile_options(-w)

add_subdirectory(${PROJECT_SOURCE_DIR}/3rdparty/tensorrtbuffer)
add_subdirectory(${PROJECT_SOURCE_DIR}/3rdparty/DBoW2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(OpenCV 4.2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CUDA REQUIRED)
find_package(yaml-cpp REQUIRED)

# CUDA link dirs (CUDA 13.0: find_package doesn't always set library paths)
link_directories(${CUDA_TOOLKIT_ROOT_DIR}/lib64 ${CUDA_TOOLKIT_ROOT_DIR}/lib)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(g2o REQUIRED)
find_package(Gflags REQUIRED)
find_package(Glog REQUIRED)

# TensorRT 10.9.0.34 — manual path (no cmake config provided)
set(TENSORRT_DIR $ENV{HOME}/lib/Deploy/TensorRT-10.9.0.34)
set(TENSORRT_INCLUDE_DIRS ${TENSORRT_DIR}/include)
set(TENSORRT_LIB_DIRS ${TENSORRT_DIR}/lib)
find_library(TENSORRT_LIBRARY_INFER nvinfer HINTS ${TENSORRT_LIB_DIRS} REQUIRED)
find_library(TENSORRT_LIBRARY_ONNX nvonnxparser HINTS ${TENSORRT_LIB_DIRS} REQUIRED)

add_library(${PROJECT_NAME}_lib SHARED
  src/g2o_optimization/vertex_line3d.cc
  src/g2o_optimization/vertex_vi_pose.cc
  src/g2o_optimization/vertex_imu.cc
  src/g2o_optimization/edge_project_point.cc
  src/g2o_optimization/edge_project_line.cc
  src/g2o_optimization/edge_imu.cc
  src/g2o_optimization/edge_relative_pose.cc
  src/g2o_optimization/g2o_optimization.cc
  src/bow/FSuperpoint.cc
  src/bow/database.cc
  src/super_point.cpp
  src/feature_detector.cc
  src/super_glue.cpp
  src/light_glue.cpp
  src/plnet.cpp
  src/utils.cc
  src/camera.cc
  src/imu.cc
  src/dataset.cc
  src/frame.cc
  src/point_matcher.cc
  src/mappoint.cc
  src/mapline.cc
  src/line_processor.cc
  src/ros_publisher.cc
  src/map.cc
  src/map_builder.cc
  src/map_refiner.cc
  src/map_user.cc
  src/timer.cc
  src/debug.cc
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
  PUBLIC
    ${YAML_CPP_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${GFLAGS_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}_lib
  PUBLIC
  rclcpp cv_bridge geometry_msgs nav_msgs
  sensor_msgs std_msgs visualization_msgs tf2_ros
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
    DBoW2
)

ament_export_include_directories(include)
# ament_export_targets — skipped (tensorrtbuffer/DBoW2 are not exported)
ament_export_dependencies(
  rclcpp
  cv_bridge
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2_ros
)

add_executable(visual_odometry demo/visual_odometry.cpp)
target_link_libraries(visual_odometry ${PROJECT_NAME}_lib curl)

add_executable(map_refinement demo/map_refinement.cpp)
target_link_libraries(map_refinement ${PROJECT_NAME}_lib curl)

add_executable(relocalization demo/relocalization.cpp)
target_link_libraries(relocalization ${PROJECT_NAME}_lib curl)

add_executable(test_feature demo/test_feature.cpp)
target_link_libraries(test_feature ${PROJECT_NAME}_lib curl)

install(TARGETS ${PROJECT_NAME}_lib tensorrtbuffer DBoW2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS visual_odometry map_refinement relocalization test_feature
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY configs/ DESTINATION share/${PROJECT_NAME}/configs)
install(DIRECTORY rviz/ voc/ launch/ DESTINATION share/${PROJECT_NAME})
install(DIRECTORY output/ DESTINATION share/${PROJECT_NAME}/output)

ament_package()
