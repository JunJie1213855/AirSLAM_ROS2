#ifndef THREAD_PUBLISHER_H_
#define THREAD_PUBLISHER_H_
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

template <class T>
class ThreadPublisher {
public:
  ~ThreadPublisher() { ShutDown(); }

  void Register(std::function<void(const std::shared_ptr<const T>&)> callback) {
    callbacks_.push_back(std::move(callback));
  }
  void Start() { thread_ = std::thread([this] { Process(); }); }
  void RequestShutdown() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      shutdown_ = true;
    }
    condition_.notify_all();
  }
  bool ShutdownRequested() {
    std::lock_guard<std::mutex> lock(mutex_);
    return shutdown_;
  }
  void Publish(const std::shared_ptr<const T> message) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (shutdown_) return;
      messages_.push(message);
    }
    condition_.notify_one();
  }
  void Process() {
    for (;;) {
      std::shared_ptr<const T> message;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] { return shutdown_ || !messages_.empty(); });
        if (messages_.empty()) break;
        message = messages_.front();
        messages_.pop();
      }
      for (const auto& callback : callbacks_) callback(message);
    }
  }
  void ShutDown() {
    RequestShutdown();
    if (thread_.joinable()) thread_.join();
  }
private:
  std::mutex mutex_;
  std::condition_variable condition_;
  std::queue<std::shared_ptr<const T>> messages_;
  std::thread thread_;
  std::vector<std::function<void(const std::shared_ptr<const T>&)>> callbacks_;
  bool shutdown_{false};
};
#endif
