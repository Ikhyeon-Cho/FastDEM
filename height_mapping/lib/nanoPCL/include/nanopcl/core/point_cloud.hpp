// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_CORE_POINT_CLOUD_HPP
#define NANOPCL_CORE_POINT_CLOUD_HPP

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "nanopcl/core/point.hpp"
#include "nanopcl/core/timestamp.hpp"

namespace nanopcl {

/// RGB color type (3 bytes)
using Color = std::array<uint8_t, 3>;

/**
 * @brief Point Cloud with metadata (frame_id, timestamp)
 *
 * Container for 3D points with associated timestamp and coordinate frame.
 * Common in robotics for sensor data (LiDAR, depth cameras, etc.).
 *
 * Design follows PCL convention: public 'points' member for direct access.
 * Usage: cloud.points[i], cloud.points.push_back(pt)
 *
 * @note Uses float precision for memory efficiency and cache performance.
 *       Sufficient for typical robotics sensors (LiDAR ~3cm precision).
 */
class PointCloud {
 public:
  // ========== Data Members (Public) ==========

  /**
   * @brief Point data storage (public for direct access)
   *
   * Following PCL convention, points are directly accessible.
   * Use cloud.points[i] to access individual points.
   */
  std::vector<Point> points;

  /**
   * @brief Default initial capacity for point clouds
   *
   * 1000 points is a reasonable default:
   * - Small enough to not waste memory for small clouds
   * - Large enough to avoid frequent reallocations
   * - Typical for filtered/clustered results
   */
  static constexpr size_t DEFAULT_CAPACITY = 10000;

  // ========== Constructors ==========

  /**
   * @brief Default constructor with default capacity
   */
  PointCloud() { points.reserve(DEFAULT_CAPACITY); }

  /**
   * @brief Construct with explicit capacity
   * @param reserve_size Initial capacity to reserve
   *
   * Example: PointCloud(10000)  // Reserve for 10000 points
   */
  explicit PointCloud(size_t reserve_size) { points.reserve(reserve_size); }

  /**
   * @brief Construct with frame ID and optional capacity
   * @param frame_id Coordinate frame (required for meaningful sensor data)
   * @param reserve_size Initial capacity (default: DEFAULT_CAPACITY)
   *
   * Examples:
   *   PointCloud cloud("laser");        // frame="laser", capacity=1000
   *   PointCloud cloud("laser", 5000);  // frame="laser", capacity=5000
   *   cloud.setTimestamp(ts);           // Set timestamp later as needed
   */
  explicit PointCloud(const std::string& frame_id,
                      size_t reserve_size = DEFAULT_CAPACITY)
      : frame_id_(frame_id) {
    points.reserve(reserve_size);
  }

  // ========== Rule of 5 (for optional channels deep copy) ==========

  /**
   * @brief Copy constructor (deep copy of all channels)
   */
  PointCloud(const PointCloud& other)
      : points(other.points),
        frame_id_(other.frame_id_),
        timestamp_ns_(other.timestamp_ns_) {
    if (other.intensity_) {
      intensity_ = std::make_unique<std::vector<float>>(*other.intensity_);
    }
    if (other.time_) {
      time_ = std::make_unique<std::vector<float>>(*other.time_);
    }
    if (other.ring_) {
      ring_ = std::make_unique<std::vector<uint16_t>>(*other.ring_);
    }
    if (other.color_) {
      color_ = std::make_unique<std::vector<Color>>(*other.color_);
    }
    if (other.label_) {
      label_ = std::make_unique<std::vector<uint32_t>>(*other.label_);
    }
  }

  /**
   * @brief Copy assignment operator (deep copy of all channels)
   */
  PointCloud& operator=(const PointCloud& other) {
    if (this != &other) {
      points = other.points;
      frame_id_ = other.frame_id_;
      timestamp_ns_ = other.timestamp_ns_;
      // Deep copy optional channels
      intensity_ = other.intensity_
          ? std::make_unique<std::vector<float>>(*other.intensity_) : nullptr;
      time_ = other.time_
          ? std::make_unique<std::vector<float>>(*other.time_) : nullptr;
      ring_ = other.ring_
          ? std::make_unique<std::vector<uint16_t>>(*other.ring_) : nullptr;
      color_ = other.color_
          ? std::make_unique<std::vector<Color>>(*other.color_) : nullptr;
      label_ = other.label_
          ? std::make_unique<std::vector<uint32_t>>(*other.label_) : nullptr;
    }
    return *this;
  }

  /**
   * @brief Move constructor
   */
  PointCloud(PointCloud&& other) noexcept = default;

  /**
   * @brief Move assignment operator
   */
  PointCloud& operator=(PointCloud&& other) noexcept = default;

  /**
   * @brief Destructor
   */
  ~PointCloud() = default;

  // ========== Merge Operator ==========

  /**
   * @brief Merge another point cloud into this one (Strict & Fast)
   *
   * Policy: Only channels already enabled in *this* are preserved.
   * - Uses vector::insert for bulk copy (memcpy-level performance)
   * - Does NOT auto-enable channels from other (schema immutability)
   * - If this has a channel but other doesn't, pads with default values
   *
   * @param other Point cloud to merge from
   * @return Reference to this cloud
   */
  PointCloud& operator+=(const PointCloud& other) {
    if (other.empty()) return *this;

    // 1. Points: bulk insert (high-speed copy via vector::insert)
    points.insert(points.end(), other.points.begin(), other.points.end());

    // 2. Channels: only process channels that *this* already has enabled
    if (intensity_) {
      if (other.intensity_) {
        intensity_->insert(intensity_->end(),
                           other.intensity_->begin(), other.intensity_->end());
      } else {
        intensity_->resize(points.size(), 0.0f);
      }
    }

    if (time_) {
      if (other.time_) {
        time_->insert(time_->end(),
                      other.time_->begin(), other.time_->end());
      } else {
        time_->resize(points.size(), 0.0f);
      }
    }

    if (ring_) {
      if (other.ring_) {
        ring_->insert(ring_->end(),
                      other.ring_->begin(), other.ring_->end());
      } else {
        ring_->resize(points.size(), 0);
      }
    }

    if (color_) {
      if (other.color_) {
        color_->insert(color_->end(),
                       other.color_->begin(), other.color_->end());
      } else {
        color_->resize(points.size(), Color{0, 0, 0});
      }
    }

    if (label_) {
      if (other.label_) {
        label_->insert(label_->end(),
                       other.label_->begin(), other.label_->end());
      } else {
        label_->resize(points.size(), 0);
      }
    }

    return *this;
  }

  // ========== Metadata Access ==========

  /**
   * @brief Get timestamp in nanoseconds
   */
  Timestamp timestamp() const { return timestamp_ns_; }

  /**
   * @brief Set timestamp in nanoseconds
   */
  void setTimestamp(Timestamp ts) { timestamp_ns_ = ts; }

  /**
   * @brief Get frame ID (coordinate frame)
   */
  const std::string& frameId() const { return frame_id_; }

  /**
   * @brief Set frame ID (coordinate frame)
   */
  void setFrameId(const std::string& id) { frame_id_ = id; }

  // ========== Intensity Channel (Optional) ==========

  /**
   * @brief Check if intensity channel is enabled
   */
  bool hasIntensity() const { return intensity_ != nullptr; }

  /**
   * @brief Enable intensity channel (lazy initialization)
   *
   * Allocates intensity storage and resizes to match current points size.
   * Safe to call multiple times (no-op if already enabled).
   */
  void enableIntensity() {
    if (!intensity_) {
      intensity_ = std::make_unique<std::vector<float>>();
      intensity_->reserve(points.capacity());
      intensity_->resize(points.size(), 0.0f);
    }
  }

  /**
   * @brief Get mutable reference to intensity vector
   * @throws std::runtime_error if intensity not enabled
   */
  std::vector<float>& intensity() {
    if (!intensity_) {
      throw std::runtime_error("Intensity channel not enabled. Call enableIntensity() first.");
    }
    return *intensity_;
  }

  /**
   * @brief Get const reference to intensity vector
   * @throws std::runtime_error if intensity not enabled
   */
  const std::vector<float>& intensity() const {
    if (!intensity_) {
      throw std::runtime_error("Intensity channel not enabled. Call enableIntensity() first.");
    }
    return *intensity_;
  }

  /**
   * @brief Disable intensity channel and free memory
   */
  void disableIntensity() { intensity_.reset(); }

  // ========== Time Channel (Optional) ==========
  // Relative timestamp per point for motion compensation (deskewing)

  bool hasTime() const { return time_ != nullptr; }

  void enableTime() {
    if (!time_) {
      time_ = std::make_unique<std::vector<float>>();
      time_->reserve(points.capacity());
      time_->resize(points.size(), 0.0f);
    }
  }

  std::vector<float>& time() {
    if (!time_) {
      throw std::runtime_error("Time channel not enabled. Call enableTime() first.");
    }
    return *time_;
  }

  const std::vector<float>& time() const {
    if (!time_) {
      throw std::runtime_error("Time channel not enabled. Call enableTime() first.");
    }
    return *time_;
  }

  void disableTime() { time_.reset(); }

  // ========== Ring Channel (Optional) ==========
  // Laser channel index for multi-layer LiDARs (ground segmentation)

  bool hasRing() const { return ring_ != nullptr; }

  void enableRing() {
    if (!ring_) {
      ring_ = std::make_unique<std::vector<uint16_t>>();
      ring_->reserve(points.capacity());
      ring_->resize(points.size(), 0);
    }
  }

  std::vector<uint16_t>& ring() {
    if (!ring_) {
      throw std::runtime_error("Ring channel not enabled. Call enableRing() first.");
    }
    return *ring_;
  }

  const std::vector<uint16_t>& ring() const {
    if (!ring_) {
      throw std::runtime_error("Ring channel not enabled. Call enableRing() first.");
    }
    return *ring_;
  }

  void disableRing() { ring_.reset(); }

  // ========== Color Channel (Optional) ==========
  // RGB color for camera-LiDAR fusion or visualization

  bool hasColor() const { return color_ != nullptr; }

  void enableColor() {
    if (!color_) {
      color_ = std::make_unique<std::vector<Color>>();
      color_->reserve(points.capacity());
      color_->resize(points.size(), Color{0, 0, 0});
    }
  }

  std::vector<Color>& color() {
    if (!color_) {
      throw std::runtime_error("Color channel not enabled. Call enableColor() first.");
    }
    return *color_;
  }

  const std::vector<Color>& color() const {
    if (!color_) {
      throw std::runtime_error("Color channel not enabled. Call enableColor() first.");
    }
    return *color_;
  }

  void disableColor() { color_.reset(); }

  // ========== Label Channel (Optional) ==========
  // Semantic label for semantic SLAM (SemanticKITTI/NuScenes format)
  // Lower 16 bits: semantic class, Upper 16 bits: instance ID

  bool hasLabel() const { return label_ != nullptr; }

  void enableLabel() {
    if (!label_) {
      label_ = std::make_unique<std::vector<uint32_t>>();
      label_->reserve(points.capacity());
      label_->resize(points.size(), 0);
    }
  }

  std::vector<uint32_t>& label() {
    if (!label_) {
      throw std::runtime_error("Label channel not enabled. Call enableLabel() first.");
    }
    return *label_;
  }

  const std::vector<uint32_t>& label() const {
    if (!label_) {
      throw std::runtime_error("Label channel not enabled. Call enableLabel() first.");
    }
    return *label_;
  }

  void disableLabel() { label_.reset(); }

  // ========== Label Helper Functions ==========

  /// Extract semantic class from label (lower 16 bits)
  static uint16_t semanticClass(uint32_t label) {
    return static_cast<uint16_t>(label & 0xFFFF);
  }

  /// Extract instance ID from label (upper 16 bits)
  static uint16_t instanceId(uint32_t label) {
    return static_cast<uint16_t>(label >> 16);
  }

  /// Create label from semantic class and instance ID
  static uint32_t makeLabel(uint16_t semantic_class, uint16_t instance_id = 0) {
    return static_cast<uint32_t>(instance_id) << 16 | semantic_class;
  }

  // ========== Convenience Methods (Wrappers) ==========

  /**
   * @brief Number of points in the cloud
   */
  size_t size() const { return points.size(); }

  /**
   * @brief Check if cloud is empty
   */
  bool empty() const { return points.empty(); }

  /**
   * @brief Reserve memory for n points (and all enabled channels)
   */
  void reserve(size_t n) {
    points.reserve(n);
    if (intensity_) intensity_->reserve(n);
    if (time_) time_->reserve(n);
    if (ring_) ring_->reserve(n);
    if (color_) color_->reserve(n);
    if (label_) label_->reserve(n);
  }

  /**
   * @brief Get allocated capacity
   */
  size_t capacity() const { return points.capacity(); }

  /**
   * @brief Resize cloud to n points (and all enabled channels)
   */
  void resize(size_t n) {
    points.resize(n);
    if (intensity_) intensity_->resize(n, 0.0f);
    if (time_) time_->resize(n, 0.0f);
    if (ring_) ring_->resize(n, 0);
    if (color_) color_->resize(n, Color{0, 0, 0});
    if (label_) label_->resize(n, 0);
  }

  /**
   * @brief Clear all points (and all enabled channels)
   */
  void clear() {
    points.clear();
    if (intensity_) intensity_->clear();
    if (time_) time_->clear();
    if (ring_) ring_->clear();
    if (color_) color_->clear();
    if (label_) label_->clear();
  }

  /**
   * @brief Add point to cloud (copy)
   * @note Enabled channels get default values (0)
   */
  void push_back(const Point& p) {
    points.push_back(p);
    if (intensity_) intensity_->push_back(0.0f);
    if (time_) time_->push_back(0.0f);
    if (ring_) ring_->push_back(0);
    if (color_) color_->push_back(Color{0, 0, 0});
    if (label_) label_->push_back(0);
  }

  /**
   * @brief Add point to cloud (move)
   * @note Enabled channels get default values (0)
   */
  void push_back(Point&& p) {
    points.push_back(std::move(p));
    if (intensity_) intensity_->push_back(0.0f);
    if (time_) time_->push_back(0.0f);
    if (ring_) ring_->push_back(0);
    if (color_) color_->push_back(Color{0, 0, 0});
    if (label_) label_->push_back(0);
  }

  /**
   * @brief Add point with intensity (synchronized)
   * @param p Point to add
   * @param intensity_val Intensity value (ignored if intensity not enabled)
   * @note Other channels get default values (0)
   */
  void push_back(const Point& p, float intensity_val) {
    points.push_back(p);
    if (intensity_) intensity_->push_back(intensity_val);
    if (time_) time_->push_back(0.0f);
    if (ring_) ring_->push_back(0);
    if (color_) color_->push_back(Color{0, 0, 0});
    if (label_) label_->push_back(0);
  }

  /**
   * @brief Construct point in-place
   * @note Enabled channels get default values (0)
   */
  template <typename... Args>
  void emplace_back(Args&&... args) {
    points.emplace_back(std::forward<Args>(args)...);
    if (intensity_) intensity_->push_back(0.0f);
    if (time_) time_->push_back(0.0f);
    if (ring_) ring_->push_back(0);
    if (color_) color_->push_back(Color{0, 0, 0});
    if (label_) label_->push_back(0);
  }

  /**
   * @brief Remove last point (and all enabled channels)
   */
  void pop_back() {
    points.pop_back();
    if (intensity_) intensity_->pop_back();
    if (time_) time_->pop_back();
    if (ring_) ring_->pop_back();
    if (color_) color_->pop_back();
    if (label_) label_->pop_back();
  }

  /**
   * @brief Access point by index
   */
  Point& operator[](size_t i) { return points[i]; }
  const Point& operator[](size_t i) const { return points[i]; }

  // ========== Iterators (for range-based for loops) ==========

  using iterator = std::vector<Point>::iterator;
  using const_iterator = std::vector<Point>::const_iterator;

  iterator begin() { return points.begin(); }
  iterator end() { return points.end(); }
  const_iterator begin() const { return points.begin(); }
  const_iterator end() const { return points.end(); }
  const_iterator cbegin() const { return points.cbegin(); }
  const_iterator cend() const { return points.cend(); }

  /**
   * @brief Erase single point (and all enabled channels)
   */
  iterator erase(iterator pos) {
    auto idx = std::distance(points.begin(), pos);
    if (intensity_) intensity_->erase(intensity_->begin() + idx);
    if (time_) time_->erase(time_->begin() + idx);
    if (ring_) ring_->erase(ring_->begin() + idx);
    if (color_) color_->erase(color_->begin() + idx);
    if (label_) label_->erase(label_->begin() + idx);
    return points.erase(pos);
  }

  /**
   * @brief Erase range of points (and all enabled channels)
   */
  iterator erase(iterator first, iterator last) {
    auto first_idx = std::distance(points.begin(), first);
    auto last_idx = std::distance(points.begin(), last);
    if (intensity_) {
      intensity_->erase(intensity_->begin() + first_idx,
                        intensity_->begin() + last_idx);
    }
    if (time_) {
      time_->erase(time_->begin() + first_idx, time_->begin() + last_idx);
    }
    if (ring_) {
      ring_->erase(ring_->begin() + first_idx, ring_->begin() + last_idx);
    }
    if (color_) {
      color_->erase(color_->begin() + first_idx, color_->begin() + last_idx);
    }
    if (label_) {
      label_->erase(label_->begin() + first_idx, label_->begin() + last_idx);
    }
    return points.erase(first, last);
  }

 private:
  std::string frame_id_;
  Timestamp timestamp_ns_{0};

  // Optional attribute channels (SoA layout for cache efficiency)
  std::unique_ptr<std::vector<float>> intensity_;    // Reflectivity
  std::unique_ptr<std::vector<float>> time_;         // Relative timestamp (deskewing)
  std::unique_ptr<std::vector<uint16_t>> ring_;      // Laser channel index
  std::unique_ptr<std::vector<Color>> color_;        // RGB color
  std::unique_ptr<std::vector<uint32_t>> label_;     // Semantic label (SemanticKITTI format)
};

}  // namespace nanopcl

#endif  // NANOPCL_CORE_POINT_CLOUD_HPP
