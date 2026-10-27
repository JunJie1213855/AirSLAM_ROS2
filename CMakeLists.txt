cmake_minimum_required(VERSION 3.5)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

project(air_slam)

set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(OpenGL_GL_PREFERENCE GLVND)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
add_compile_options(-w)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_srvs REQUIRED)

find_package(OpenCV 4.2 REQUIRED)
find_package(Eigen3 REQUIRED)
if(CUDA_ROOT)
  set(CUDA_TOOLKIT_ROOT_DIR "${CUDA_ROOT}" CACHE PATH "CUDA toolkit root" FORCE)
endif()
find_package(CUDA REQUIRED)
find_package(yaml-cpp REQUIRED)

# CUDA link dirs (CUDA 13.0: find_package doesn't always set library paths)
link_directories(${CUDA_TOOLKIT_ROOT_DIR}/lib64 ${CUDA_TOOLKIT_ROOT_DIR}/lib)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(g2o REQUIRED)
find_package(Gflags REQUIRED)
find_package(Glog REQUIRED)

# Share one SDK across AirSLAM and its buffer library.
set(TensorRT_ROOT "$ENV{TensorRT_ROOT}" CACHE PATH "TensorRT SDK root")
if(NOT TensorRT_ROOT)
  set(TensorRT_ROOT "$ENV{HOME}/lib/TensorRT-11.3.0.99")
endif()
find_path(TENSORRT_INCLUDE_DIRS NvInfer.h HINTS "${TensorRT_ROOT}/include" REQUIRED NO_DEFAULT_PATH)
find_library(TENSORRT_LIBRARY_INFER nvinfer HINTS "${TensorRT_ROOT}/lib" "${TensorRT_ROOT}/lib64" REQUIRED NO_DEFAULT_PATH)
find_library(TENSORRT_LIBRARY_ONNX nvonnxparser HINTS "${TensorRT_ROOT}/lib" "${TensorRT_ROOT}/lib64" REQUIRED NO_DEFAULT_PATH)
get_filename_component(TENSORRT_LIB_DIRS "${TENSORRT_LIBRARY_INFER}" DIRECTORY)
set(CMAKE_INSTALL_RPATH "${TENSORRT_LIB_DIRS};${CUDA_TOOLKIT_ROOT_DIR}/lib64")
message(STATUS "AirSLAM TensorRT SDK: ${TensorRT_ROOT}")
add_subdirectory(${PROJECT_SOURCE_DIR}/3rdparty/tensorrtbuffer)
add_subdirectory(${PROJECT_SOURCE_DIR}/3rdparty/DBoW2)

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
  src/feature_selection.cc
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

add_executable(stereo_inertial_node demo/stereo_inertial_node.cpp)
target_link_libraries(stereo_inertial_node ${PROJECT_NAME}_lib)
ament_target_dependencies(stereo_inertial_node message_filters std_srvs)

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

install(TARGETS visual_odometry stereo_inertial_node map_refinement relocalization test_feature
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY configs/ DESTINATION share/${PROJECT_NAME}/configs)
install(DIRECTORY rviz/ voc/ launch/ DESTINATION share/${PROJECT_NAME})
install(DIRECTORY output/ DESTINATION share/${PROJECT_NAME}/output)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_thread_publisher test/thread_publisher_test.cpp)
  target_include_directories(test_thread_publisher PRIVATE include)
  target_link_libraries(test_thread_publisher pthread)
  ament_add_gtest(test_feature_selection test/feature_selection_test.cpp)
  target_link_libraries(test_feature_selection ${PROJECT_NAME}_lib)
  ament_add_gtest(test_imu_preintegration test/imu_preintegration_test.cpp)
  target_link_libraries(test_imu_preintegration ${PROJECT_NAME}_lib)
endif()

ament_package()
