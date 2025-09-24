/*
 * ros_param_watcher.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ROS_PARAM_WATCHER_H
#define ROS_PARAM_WATCHER_H

#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>
#include <ros/ros.h>
#include <string>
#include <unordered_map>
#include <variant>

namespace ros_utils {

/**
 * @brief General-purpose ROS parameter watcher
 *
 * Lightweight alternative to dynamic_reconfigure that polls parameters
 * and triggers callbacks on changes. Project-agnostic and reusable.
 *
 * Example usage:
 *   ros_utils::RosParamWatcher watcher(nh);
 *   watcher.watch("my_param", [](double val) { std::cout << val; });
 */
class RosParamWatcher {
public:
  explicit RosParamWatcher(::ros::NodeHandle &nh, double poll_rate_hz = 1.0)
      : nh_(nh) {
    timer_ = nh_.createTimer(::ros::Duration(1.0 / poll_rate_hz),
                             &RosParamWatcher::checkAll, this);
  }

  // Generic watch function - type deduced from callback
  template <typename T>
  void watch(const std::string &param_name, std::function<void(T)> callback) {
    // Create watcher and get reference
    auto result = params_.emplace(param_name, ParamWatcher<T>(param_name, callback));

    // Initialize with current value if exists
    T value;
    if (nh_.getParam(param_name, value)) {
      // Access the specific watcher and set initialized state
      auto& watcher = std::get<ParamWatcher<T>>(result.first->second);
      watcher.last_value = value;
      watcher.initialized = true;
      callback(value);
    }
  }

  // Manual check (useful for testing)
  void checkNow() { checkAll(::ros::TimerEvent()); }

private:
  template <typename T> struct ParamWatcher {
    std::string name;
    std::function<void(T)> callback;
    T last_value{};
    bool initialized = false;

    // Default constructor needed for std::variant
    ParamWatcher() = default;

    // Constructor with parameters
    ParamWatcher(const std::string& n, std::function<void(T)> cb)
      : name(n), callback(cb) {}

    void check(::ros::NodeHandle &nh) {
      T current;
      if (nh.getParam(name, current)) {
        if (!initialized || !isEqual(current, last_value)) {
          if (initialized) {
            ROS_INFO("Parameter '%s' changed", name.c_str());
          }
          last_value = current;
          initialized = true;
          callback(current);
        }
      }
    }

  private:
    // Helper for equality comparison - general case
    template<typename U>
    static bool isEqual(const U& a, const U& b) {
      return a == b;
    }

    // Specialization for double comparison
    static bool isEqual(double a, double b) {
      return std::abs(a - b) < 1e-9;
    }
  };

  void checkAll(const ::ros::TimerEvent & /*event*/) {
    for (auto &[name, watcher] : params_) {
      std::visit([this](auto &w) { w.check(nh_); }, watcher);
    }
  }

  ::ros::NodeHandle &nh_;
  ::ros::Timer timer_;

  using WatcherVariant =
      std::variant<ParamWatcher<std::string>, ParamWatcher<int>,
                   ParamWatcher<double>, ParamWatcher<bool>>;
  std::unordered_map<std::string, WatcherVariant> params_;
};

} // namespace ros_utils

// Project-specific extensions for height_mapping
namespace height_mapping::ros {

// Import the generic watcher
using ::ros_utils::RosParamWatcher;

/**
 * @brief Setup logger parameter watching for this project
 */
inline void setupLoggerParams(RosParamWatcher &watcher) {
// Only include logger if available
#ifdef LOGGER_LOGGER_H
  watcher.watch<std::string>("logger/level", [](const std::string &level) {
    // Convert to uppercase for case-insensitive comparison
    std::string upper_level = level;
    std::transform(upper_level.begin(), upper_level.end(), upper_level.begin(), ::toupper);

    if (upper_level == "DEBUG")
      logger::Logger::setLevel(logger::DEBUG);
    else if (upper_level == "INFO")
      logger::Logger::setLevel(logger::INFO);
    else if (upper_level == "WARN" || upper_level == "WARNING")
      logger::Logger::setLevel(logger::WARN);
    else if (upper_level == "ERROR")
      logger::Logger::setLevel(logger::ERROR);
    else {
      // Log warning for invalid level
      LOG_WARN("ParamWatcher", "Invalid logger level '", level, "'. Using INFO. Valid: DEBUG, INFO, WARN, ERROR");
      logger::Logger::setLevel(logger::INFO);
    }
  });
#else
  // Fallback to ROS logging
  ROS_WARN_ONCE("Logger not available, using ROS logging");
#endif
}

} // namespace height_mapping::ros

#endif // ROS_PARAM_WATCHER_H