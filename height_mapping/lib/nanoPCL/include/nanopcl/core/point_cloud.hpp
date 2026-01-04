// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_CORE_POINT_CLOUD_HPP
#define NANOPCL_CORE_POINT_CLOUD_HPP

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "nanopcl/core/point.hpp"
#include "nanopcl/core/point_accessors.hpp"
#include "nanopcl/core/timestamp.hpp"

namespace nanopcl {

/**
 * @brief Point Cloud with metadata and optional attribute channels
 *
 * Container for 3D points with timestamp, coordinate frame, and optional
 * per-point attributes (intensity, time, ring, color, label).
 *
 * @note Uses float precision for memory efficiency and cache performance.
 */
class PointCloud {
 public:
  static constexpr size_t DEFAULT_CAPACITY = 65536;

  // ==========================================================================
  // Type Aliases (from point_accessors.hpp)
  // ==========================================================================
  using PointRef = nanopcl::PointRef;
  using ConstPointRef = nanopcl::ConstPointRef;

  using iterator = nanopcl::PointRefIterator;
  using const_iterator = nanopcl::ConstPointRefIterator;
  using reverse_iterator = std::reverse_iterator<iterator>;
  using const_reverse_iterator = std::reverse_iterator<const_iterator>;

  // ==========================================================================
  // Lifecycle
  // ==========================================================================

  PointCloud();
  explicit PointCloud(size_t reserve_size);
  explicit PointCloud(const std::string& frame_id,
                      size_t reserve_size = DEFAULT_CAPACITY);

  PointCloud(const PointCloud& other);
  PointCloud& operator=(const PointCloud& other);
  PointCloud(PointCloud&& other) noexcept = default;
  PointCloud& operator=(PointCloud&& other) noexcept = default;
  ~PointCloud() = default;

  // ==========================================================================
  // Metadata & Global Operations
  // ==========================================================================

  [[nodiscard]] Timestamp timestamp() const { return timestamp_ns_; }
  void setTimestamp(Timestamp ts) { timestamp_ns_ = ts; }

  [[nodiscard]] const std::string& frameId() const { return frame_id_; }
  void setFrameId(const std::string& id) { frame_id_ = id; }

  /// Merge another point cloud (only preserved channels are copied)
  PointCloud& operator+=(const PointCloud& other);

  /// Merge two point clouds and return a new one
  [[nodiscard]] PointCloud operator+(const PointCloud& other) const;

  // ==========================================================================
  // Attribute Channels
  // ==========================================================================

  // --- Intensity (Reflectivity) ---
  [[nodiscard]] bool hasIntensity() const { return intensity_ != nullptr; }
  void enableIntensity();
  std::vector<float>& intensity();
  const std::vector<float>& intensity() const;
  void disableIntensity() { intensity_.reset(); }

  // --- Time (Relative timestamp) ---
  [[nodiscard]] bool hasTime() const { return time_ != nullptr; }
  void enableTime();
  std::vector<float>& time();
  const std::vector<float>& time() const;
  void disableTime() { time_.reset(); }

  // --- Ring (Laser channel index) ---
  [[nodiscard]] bool hasRing() const { return ring_ != nullptr; }
  void enableRing();
  std::vector<uint16_t>& ring();
  const std::vector<uint16_t>& ring() const;
  void disableRing() { ring_.reset(); }

  // --- Color (RGB storage) ---
  [[nodiscard]] bool hasColor() const { return color_ != nullptr; }
  void enableColor();
  std::vector<Color>& color();
  const std::vector<Color>& color() const;
  void disableColor() { color_.reset(); }

  // --- Normal (Surface normal vector) ---
  [[nodiscard]] bool hasNormal() const { return normal_ != nullptr; }
  void enableNormal();
  std::vector<Eigen::Vector3f>& normal();
  const std::vector<Eigen::Vector3f>& normal() const;
  void disableNormal() { normal_.reset(); }

  // --- Label (Semantic segmentation) ---
  [[nodiscard]] bool hasLabel() const { return label_ != nullptr; }
  void enableLabel();
  std::vector<Label>& label();
  const std::vector<Label>& label() const;
  void disableLabel() { label_.reset(); }

  // ==========================================================================
  // Data Access & Storage
  // ==========================================================================

  // --- Container Info ---
  [[nodiscard]] size_t size() const { return points_.size(); }
  [[nodiscard]] bool empty() const { return points_.empty(); }
  [[nodiscard]] size_t capacity() const { return points_.capacity(); }
  void reserve(size_t n);
  void resize(size_t n);
  void shrink_to_fit();
  void clear();
  void swap(PointCloud& other) noexcept;

  // --- Point Addition ---
  void push_back(const Point& p);
  void push_back(Point&& p);
  void pop_back();

  /**
   * @brief Add point with attributes (variadic, type-safe)
   * Channels are auto-enabled when attributes are provided.
   * Usage: cloud.push_back(pt, Intensity(0.5f), Ring(3));
   */
  template <typename... Attrs>
  void push_back(const Point& p, Attrs&&... attrs) {
    push_back(p);
    (applyAttribute(std::forward<Attrs>(attrs)), ...);
  }

  // --- XYZ (Coordinates) ---
  std::vector<Point>& xyz() { return points_; }
  const std::vector<Point>& xyz() const { return points_; }

  // --- Data Access (STL-compatible) ---
  Point& operator[](size_t i) { return points_[i]; }
  const Point& operator[](size_t i) const { return points_[i]; }

  /// @brief Extract subset by indices (Fancy Indexing)
  /// @param indices Vector of point indices to extract
  /// @return New PointCloud containing only the specified points with all
  /// attributes
  [[nodiscard]] PointCloud operator[](const std::vector<size_t>& indices) const;

  Point* data() { return points_.data(); }
  const Point* data() const { return points_.data(); }

  Point& at(size_t i) { return points_.at(i); }
  const Point& at(size_t i) const { return points_.at(i); }

  Point& front() { return points_.front(); }
  const Point& front() const { return points_.front(); }
  Point& back() { return points_.back(); }
  const Point& back() const { return points_.back(); }

  // --- Iterators ---
  iterator begin() { return iterator(*this, 0); }
  iterator end() { return iterator(*this, size()); }
  const_iterator begin() const { return const_iterator(*this, 0); }
  const_iterator end() const { return const_iterator(*this, size()); }
  const_iterator cbegin() const { return const_iterator(*this, 0); }
  const_iterator cend() const { return const_iterator(*this, size()); }
  reverse_iterator rbegin() { return reverse_iterator(end()); }
  reverse_iterator rend() { return reverse_iterator(begin()); }
  const_reverse_iterator rbegin() const {
    return const_reverse_iterator(end());
  }
  const_reverse_iterator rend() const {
    return const_reverse_iterator(begin());
  }
  const_reverse_iterator crbegin() const {
    return const_reverse_iterator(cend());
  }
  const_reverse_iterator crend() const {
    return const_reverse_iterator(cbegin());
  }

  // --- Erase Operations ---
  iterator erase(iterator pos);
  iterator erase(iterator first, iterator last);
  void erase(size_t index);
  void erase(size_t first, size_t last);

  // --- Point-Centric Access (xyz + all attributes) ---
  PointRef point(size_t i) { return PointRef(*this, i); }
  ConstPointRef point(size_t i) const { return ConstPointRef(*this, i); }

 private:
  // Friend declarations for accessor classes
  template <bool>
  friend class PointRefImpl;
  template <bool>
  friend class PointRefIteratorImpl;

  // Attribute Application Helpers
  void applyAttribute(const Intensity& v) {
    if (!intensity_) enableIntensity();
    intensity_->back() = v.val;
  }
  void applyAttribute(const Time& v) {
    if (!time_) enableTime();
    time_->back() = v.val;
  }
  void applyAttribute(const Ring& v) {
    if (!ring_) enableRing();
    ring_->back() = v.val;
  }
  void applyAttribute(const Color& v) {
    if (!color_) enableColor();
    color_->back() = v;
  }
  void applyAttribute(const Label& v) {
    if (!label_) enableLabel();
    label_->back() = v;
  }
  void applyAttribute(const Normal& v) {
    if (!normal_) enableNormal();
    normal_->back() = v.val;
  }

  // Data Members
  std::vector<Point> points_;
  std::string frame_id_;
  Timestamp timestamp_ns_{0};

  std::unique_ptr<std::vector<float>> intensity_;
  std::unique_ptr<std::vector<float>> time_;
  std::unique_ptr<std::vector<uint16_t>> ring_;
  std::unique_ptr<std::vector<Color>> color_;
  std::unique_ptr<std::vector<Label>> label_;
  std::unique_ptr<std::vector<Eigen::Vector3f>> normal_;
};

}  // namespace nanopcl

#include "nanopcl/core/impl/point_accessors_impl.hpp"
#include "nanopcl/core/impl/point_cloud_impl.hpp"

#endif  // NANOPCL_CORE_POINT_CLOUD_HPP
