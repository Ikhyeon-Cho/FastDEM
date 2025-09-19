/*
 * data_frame.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_PIPELINE_PROCESSING_CONTEXT_H
#define HEIGHT_MAPPING_CORE_PIPELINE_PROCESSING_CONTEXT_H

#include <any>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

#include "height_map_core/height_map.h"
#include "height_mapping_core/data/point_cloud.h"
#include "height_mapping_core/data/transform.h"

namespace height_mapping::core {

// Processing context that flows through pipeline stages
class ProcessingContext {
public:
  struct Context {

    // Point cloud being processed through pipeline
    PointCloudXYZ::Ptr cloud;

    // Map & Transforms
    std::shared_ptr<height_map::HeightMap> map;
    Transform3D sensor_to_map;
    Transform3D robot_to_map;

    // Note: timestamp and frame_id are stored in cloud
    // to avoid duplication. Use cloud().timestamp and cloud().frame_id

    Context() {
      cloud = PointCloudXYZ::create();
      map = std::make_shared<height_map::HeightMap>();
    }
  };

  // Constructor
  ProcessingContext() : context_(std::make_unique<Context>()) {}

  // Access primary context
  Context &context() { return *context_; }
  const Context &context() const { return *context_; }

  // Convenience accessors
  PointCloudXYZ &cloud() { return *context_->cloud; }
  const PointCloudXYZ &cloud() const { return *context_->cloud; }

  height_map::HeightMap &map() { return *context_->map; }
  const height_map::HeightMap &map() const { return *context_->map; }

  // Shared pointer accessors (for ownership transfer)
  PointCloudXYZ::Ptr getCloudPtr() { return context_->cloud; }
  std::shared_ptr<height_map::HeightMap> getMapPtr() { return context_->map; }

  // Setters for cleaner assignment
  void setMap(std::shared_ptr<height_map::HeightMap> map) { context_->map = map; }
  void setCloudPtr(PointCloudXYZ::Ptr cloud) { context_->cloud = cloud; }

  // Metadata storage (for stage-specific data)
  template <typename T> void set(const std::string &key, T &&value) {
    metadata_[key] = std::forward<T>(value);
  }

  template <typename T> std::optional<T> get(const std::string &key) const {
    auto it = metadata_.find(key);
    if (it != metadata_.end()) {
      try {
        return std::any_cast<T>(it->second);
      } catch (const std::bad_any_cast &) {
        return std::nullopt;
      }
    }
    return std::nullopt;
  }

  template <typename T>
  T getOrDefault(const std::string &key, const T &default_value) const {
    return get<T>(key).value_or(default_value);
  }

  bool has(const std::string &key) const {
    return metadata_.find(key) != metadata_.end();
  }

  void remove(const std::string &key) { metadata_.erase(key); }

  void clearMetadata() { metadata_.clear(); }

  // Processing flags
  enum class ProcessingMode {
    REALTIME, // Fast processing for real-time operation
    ACCURATE, // Accurate processing (may be slower)
    BALANCED  // Balance between speed and accuracy
  };

  void setProcessingMode(ProcessingMode mode) { mode_ = mode; }
  ProcessingMode getProcessingMode() const { return mode_; }

  // Stage communication
  struct StageMessage {
    std::string from_stage;
    std::string to_stage;
    std::any data;
  };

  void sendMessage(const std::string &from, const std::string &to,
                   std::any data) {
    messages_.push_back({from, to, std::move(data)});
  }

  std::vector<StageMessage> getMessagesFor(const std::string &stage) const {
    std::vector<StageMessage> result;
    for (const auto &msg : messages_) {
      if (msg.to_stage == stage || msg.to_stage.empty()) {
        result.push_back(msg);
      }
    }
    return result;
  }

  void clearMessages() { messages_.clear(); }

  // Error handling
  void setError(const std::string &stage, const std::string &error) {
    errors_[stage] = error;
    has_error_ = true;
  }

  bool hasError() const { return has_error_; }

  std::string getError(const std::string &stage) const {
    auto it = errors_.find(stage);
    return (it != errors_.end()) ? it->second : "";
  }

  std::unordered_map<std::string, std::string> getAllErrors() const {
    return errors_;
  }

  void clearErrors() {
    errors_.clear();
    has_error_ = false;
  }

  // Statistics
  struct Statistics {
    size_t points_processed = 0;
    size_t points_filtered = 0;
    size_t cells_updated = 0;
    double processing_time_ms = 0.0;

    void reset() {
      points_processed = 0;
      points_filtered = 0;
      cells_updated = 0;
      processing_time_ms = 0.0;
    }
  };

  Statistics &stats() { return stats_; }
  const Statistics &stats() const { return stats_; }

  // Reset context for new processing cycle
  void reset() {
    // Don't reset the map (it's persistent)
    context_->cloud->clear();
    context_->sensor_to_map = Transform3D::Identity();
    context_->robot_to_map = Transform3D::Identity();
    // Note: timestamp and frame_id are in the cloud, which we just cleared

    clearMetadata();
    clearMessages();
    clearErrors();
    stats_.reset();
  }

private:
  std::unique_ptr<Context> context_;
  std::unordered_map<std::string, std::any> metadata_;
  std::vector<StageMessage> messages_;
  std::unordered_map<std::string, std::string> errors_;
  bool has_error_ = false;
  ProcessingMode mode_ = ProcessingMode::BALANCED;
  Statistics stats_;
};

} // namespace height_mapping::core

#endif // HEIGHT_MAPPING_CORE_PIPELINE_PROCESSING_CONTEXT_H