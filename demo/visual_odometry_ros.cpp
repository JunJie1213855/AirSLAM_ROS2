/**
*
*/

#include <iostream>
#include <chrono>
#include <functional>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

// AirVO
#include "read_configs.h"
#include "dataset.h"
#include "map_builder.h"

MapBuilder* p_map_builder;

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////

void GrabStereo(const sensor_msgs::msg::Image::ConstSharedPtr& imgLeft,
                const sensor_msgs::msg::Image::ConstSharedPtr& imgRight){
    // Copy the ros image messages to cvMat
    cv_bridge::CvImageConstPtr cv_ptrLeft, cv_ptrRight;
    try{
        cv_ptrLeft = cv_bridge::toCvShare(imgLeft);
        cv_ptrRight = cv_bridge::toCvShare(imgRight);
    }
    catch (cv_bridge::Exception& e){
        RCLCPP_ERROR(rclcpp::get_logger("air_vo_ros"), "cv_bridge exception: %s", e.what());
        return;
    }

    static int frame_id = 0;
    auto before_infer = std::chrono::steady_clock::now();

    InputDataPtr input_data = std::shared_ptr<InputData>(new InputData());
    input_data->index = frame_id;
    input_data->image_left = cv_ptrLeft->image.clone();
    input_data->image_right = cv_ptrRight->image.clone();
    input_data->time = rclcpp::Time(imgLeft->header.stamp).seconds();

    p_map_builder->AddInput(input_data);

    auto after_infer = std::chrono::steady_clock::now();
    auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>
    (after_infer - before_infer).count();

    std::cout << "i ===== " << frame_id++ << " Processing Time: " << cost_time << " ms." << std::endl;
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("air_vo_ros");

    // ROS
    std::string left_topic, right_topic;
    node->declare_parameter<std::string>("left_topic", "/cam0/image_raw");
    node->get_parameter("left_topic", left_topic);
    node->declare_parameter<std::string>("right_topic", "/cam1/image_raw");
    node->get_parameter("right_topic", right_topic);

    // AirVO
    std::string config_path, model_dir;
    node->declare_parameter<std::string>("config_path", "");
    node->get_parameter("config_path", config_path);
    node->declare_parameter<std::string>("model_dir", "");
    node->get_parameter("model_dir", model_dir);
    Configs configs(config_path, model_dir);
    node->declare_parameter<std::string>("camera_config_path", configs.camera_config_path);
    node->get_parameter("camera_config_path", configs.camera_config_path);
    node->declare_parameter<std::string>("saving_dir", configs.saving_dir);
    node->get_parameter("saving_dir", configs.saving_dir);
    std::string traj_path;
    node->declare_parameter<std::string>("traj_path", ConcatenateFolderAndFileName(configs.saving_dir, "traj.txt"));
    node->get_parameter("traj_path", traj_path);

    p_map_builder = new MapBuilder(configs, node);

    message_filters::Subscriber<sensor_msgs::msg::Image> sub_img_left(
        node.get(), left_topic, rmw_qos_profile_sensor_data);
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_img_right(
        node.get(), right_topic, rmw_qos_profile_sensor_data);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), sub_img_left, sub_img_right);
    sync.registerCallback(&GrabStereo);

    // Starts the operation
    rclcpp::spin(node);

    // Shutting down
    std::cout << "Saving trajectory to " << traj_path << std::endl;
    p_map_builder->ShutDown();
    p_map_builder->SaveTrajectory(traj_path);

    rclcpp::shutdown();

    return 0;
}
