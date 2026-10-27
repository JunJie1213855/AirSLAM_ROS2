#include <atomic>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <mutex>
#include <thread>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "map_builder.h"

// Feed the existing frontend from standard ROS image/IMU messages.
// A worker waits for IMU coverage; ROS callbacks never block on GPU inference.
class StereoInertialNode : public rclcpp::Node {
  using Image = sensor_msgs::msg::Image;
  using Imu = sensor_msgs::msg::Imu;
  using SyncPolicy = message_filters::sync_policies::ExactTime<Image, Image>;
  struct Pair { Image::ConstSharedPtr left, right; };
public:
  StereoInertialNode() : Node("airslam_stereo_inertial") {}
  void Initialize() {
    const auto config = declare_parameter<std::string>("config_path", "");
    const auto models = declare_parameter<std::string>("model_dir", "");
    VisualOdometryConfigs configs(config, models);
    configs.camera_config_path = declare_parameter<std::string>("camera_config_path", "");
    saving_dir_ = declare_parameter<std::string>("saving_dir", "");
    if (saving_dir_.empty()) throw std::invalid_argument("saving_dir must be set");
    std::filesystem::create_directories(saving_dir_);
    configs.saving_dir = saving_dir_;
    queue_capacity_ = declare_parameter<int>("max_pending_frames", 40);
    if (queue_capacity_ <= 0) throw std::invalid_argument("max_pending_frames must be positive");
    builder_ = std::make_unique<MapBuilder>(configs, shared_from_this());
    use_imu_ = builder_->UseIMU();
    const auto qos = rclcpp::SensorDataQoS().keep_last(100);
    left_.subscribe(this, "cam0_image", qos.get_rmw_qos_profile());
    right_.subscribe(this, "cam1_image", qos.get_rmw_qos_profile());
    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(100), left_, right_);
    sync_->registerCallback(std::bind(&StereoInertialNode::OnStereo, this,
                                      std::placeholders::_1, std::placeholders::_2));
    imu_ = create_subscription<Imu>("imu", qos, [this](Imu::ConstSharedPtr message) {
      ImuData value;
      value.timestamp = rclcpp::Time(message->header.stamp).seconds();
      value.gyr << message->angular_velocity.x, message->angular_velocity.y, message->angular_velocity.z;
      value.acc << message->linear_acceleration.x, message->linear_acceleration.y, message->linear_acceleration.z;
      std::lock_guard<std::mutex> lock(mutex_);
      if (finishing_) return;
      if (!imu_buffer_.empty() && value.timestamp <= imu_buffer_.back().timestamp) return;
      imu_buffer_.push_back(value);
      if (imu_buffer_.size() > 20000) imu_buffer_.pop_front();
      condition_.notify_one();
    });
    finish_ = create_service<std_srvs::srv::Trigger>("airslam/finish",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          try {
            Finalize();
            response->success = worker_error_.empty();
            response->message = worker_error_.empty()
                ? "Saved trajectory/map; submitted=" + std::to_string(submitted_) +
                  ", dropped=" + std::to_string(dropped_) : worker_error_;
          } catch (const std::exception& error) {
            response->success = false;
            response->message = error.what();
          }
        });
    worker_ = std::thread([this] { Process(); });
    RCLCPP_INFO(get_logger(), "Ready for synchronized stereo and IMU; use /airslam/finish after playback");
  }
  ~StereoInertialNode() override {
    try { Finalize(); } catch (const std::exception& error) {
      RCLCPP_ERROR(get_logger(), "Finalization failed: %s", error.what());
    }
  }
private:
  void OnStereo(const Image::ConstSharedPtr& left, const Image::ConstSharedPtr& right) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (finishing_) return;
    if (static_cast<int>(pending_.size()) >= queue_capacity_) {
      ++dropped_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                          "Input queue full; slow bag playback (dropped=%zu)", dropped_);
      return;
    }
    pending_.push_back({left, right});
    condition_.notify_one();
  }
  void Process() {
    try {
      for (;;) {
        Pair pair;
        ImuDataList batch;
        {
          std::unique_lock<std::mutex> lock(mutex_);
          condition_.wait(lock, [this] {
            if (finishing_) return true;
            if (pending_.empty()) return false;
            const double stamp = rclcpp::Time(pending_.front().left->header.stamp).seconds();
            return !use_imu_ || (!imu_buffer_.empty() && imu_buffer_.back().timestamp > stamp);
          });
          if (pending_.empty()) {
            if (finishing_) break;
            continue;
          }
          pair = pending_.front();
          pending_.pop_front();
          const double stamp = rclcpp::Time(pair.left->header.stamp).seconds();
          if (stamp <= last_stamp_) { ++dropped_; continue; }
          if (use_imu_) {
            if (imu_buffer_.empty() || imu_buffer_.front().timestamp > stamp ||
                imu_buffer_.back().timestamp < stamp) {
              ++dropped_;
              continue;
            }
            // Preserve the bracketing IMU sample around the previous image.
            while (imu_buffer_.size() > 1 && imu_buffer_[1].timestamp <= last_stamp_)
              imu_buffer_.pop_front();
            for (const auto& value : imu_buffer_) {
              batch.push_back(value);
              if (value.timestamp > stamp) break;
            }
          }
          last_stamp_ = stamp;
        }
        auto input = std::make_shared<InputData>();
        input->index = submitted_++;
        input->time = rclcpp::Time(pair.left->header.stamp).seconds();
        input->image_left = cv_bridge::toCvCopy(pair.left, "mono8")->image;
        input->image_right = cv_bridge::toCvCopy(pair.right, "mono8")->image;
        input->batch_imu_data = std::move(batch);
        builder_->AddInput(input);
      }
    } catch (const std::exception& error) {
      worker_error_ = error.what();
      RCLCPP_ERROR(get_logger(), "Input worker failed: %s", error.what());
    }
  }
  void Finalize() {
    if (!builder_ || finalized_) return;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      finishing_ = true;
    }
    condition_.notify_all();
    if (worker_.joinable()) worker_.join();
    builder_->Stop();
    finalized_ = true;
    builder_->SaveTrajectory(saving_dir_ + "/keyframe_trajectory.txt");
    builder_->SaveMap(saving_dir_);
    RCLCPP_INFO(get_logger(), "Saved results; submitted=%zu, dropped=%zu", submitted_, dropped_);
  }
  std::unique_ptr<MapBuilder> builder_;
  message_filters::Subscriber<Image> left_, right_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Subscription<Imu>::SharedPtr imu_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr finish_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<Pair> pending_;
  std::deque<ImuData> imu_buffer_;
  std::thread worker_;
  bool finishing_{false}, finalized_{false}, use_imu_{true};
  int queue_capacity_{40};
  size_t submitted_{0}, dropped_{0};
  double last_stamp_{-1};
  std::string saving_dir_, worker_error_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<StereoInertialNode>();
    node->Initialize();
    rclcpp::spin(node);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("airslam_stereo_inertial"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
