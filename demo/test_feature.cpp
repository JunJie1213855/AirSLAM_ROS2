#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include "read_configs.h"
#include "super_point.h"
#include "point_matching.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("test_feature");

  std::string config_path, model_dir, image0_path, image1_path;
  nh->declare_parameter<std::string>("config_path", "");
  nh->get_parameter("config_path", config_path);
  nh->declare_parameter<std::string>("model_dir", "");
  nh->get_parameter("model_dir", model_dir);
  nh->declare_parameter<std::string>("image0", "");
  nh->get_parameter("image0", image0_path);
  nh->declare_parameter<std::string>("image1", "");
  nh->get_parameter("image1", image1_path);

  Configs configs(config_path, model_dir);
  SuperPoint superpoint(configs.superpoint_config);
  if(!superpoint.build()){
    std::cout << "Error in SuperPoint building" << std::endl;
    return 1;
  }

  cv::Mat image0 = cv::imread(image0_path, 0);
  if(image0.empty()){
    std::cout << "cannot read image0: " << image0_path << std::endl;
    return 1;
  }
  Eigen::Matrix<double, 259, Eigen::Dynamic> features0, features1;
  if(!superpoint.infer(image0, features0)){
    std::cout << "SuperPoint inference failed on image0" << std::endl;
    return 1;
  }
  std::cout << "image0 features = " << features0.cols() << std::endl;

  if(image1_path.empty()){
    features1 = features0;
    std::cout << "using identical features for image1" << std::endl;
  }else{
    cv::Mat image1 = cv::imread(image1_path, 0);
    if(image1.empty()){
      std::cout << "cannot read image1: " << image1_path << std::endl;
      return 1;
    }
    if(!superpoint.infer(image1, features1)){
      std::cout << "SuperPoint inference failed on image1" << std::endl;
      return 1;
    }
    std::cout << "image1 features = " << features1.cols() << std::endl;
  }

  PointMatching point_matching(configs.superglue_config);
  std::vector<cv::DMatch> matches;
  int num_matches = point_matching.MatchingPoints(features0, features1, matches);
  std::cout << "matches = " << num_matches << std::endl;

  int diagonal = 0;
  for(size_t i = 0; i < matches.size(); ++i){
    if(matches[i].queryIdx == matches[i].trainIdx) diagonal++;
  }
  std::cout << "diagonal matches = " << diagonal << std::endl;

  for(size_t i = 0; i < matches.size() && i < 5; ++i){
    int l = matches[i].queryIdx, r = matches[i].trainIdx;
    std::cout << "match " << l << "->" << r << " L(" << features0(1,l) << ","
              << features0(2,l) << ") R(" << features1(1,r) << ","
              << features1(2,r) << ")" << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}
