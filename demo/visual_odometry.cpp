#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include "read_configs.h"
#include "dataset.h"
#include "map_builder.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("air_vo");

  std::string config_path, model_dir;
  nh->declare_parameter<std::string>("config_path", "");
  nh->get_parameter("config_path", config_path);
  nh->declare_parameter<std::string>("model_dir", "");
  nh->get_parameter("model_dir", model_dir);
  Configs configs(config_path, model_dir);
  std::cout << "config done" << std::endl;

  nh->declare_parameter<std::string>("dataroot", configs.dataroot);
  nh->get_parameter("dataroot", configs.dataroot);
  nh->declare_parameter<std::string>("camera_config_path", configs.camera_config_path);
  nh->get_parameter("camera_config_path", configs.camera_config_path);
  nh->declare_parameter<std::string>("saving_dir", configs.saving_dir);
  nh->get_parameter("saving_dir", configs.saving_dir);
  std::string traj_path;
  nh->declare_parameter<std::string>("traj_path", ConcatenateFolderAndFileName(configs.saving_dir, "traj.txt"));
  nh->get_parameter("traj_path", traj_path);

  Dataset dataset(configs.dataroot);
  MapBuilder map_builder(configs, nh);
  size_t dataset_length = dataset.GetDatasetLength();
  std::cout << "dataset done" << std::endl;

  for(size_t i = 0; i < dataset_length && rclcpp::ok(); ++i){
    std::cout << "i ====== " << i << std::endl;
    auto before_infer = std::chrono::steady_clock::now();

    InputDataPtr input_data = dataset.GetData(i);
    if(input_data == nullptr) continue;
    map_builder.AddInput(input_data);

    auto after_infer = std::chrono::steady_clock::now();
    auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(after_infer - before_infer).count();
    std::cout << "One Frame Processinh Time: " << cost_time << " ms." << std::endl;
  }
  map_builder.ShutDown();
  map_builder.SaveTrajectory(traj_path);
  rclcpp::shutdown();

  return 0;
}
