/*
 * point_cloud.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_GEOMETRY_POINT_CLOUD_H
#define HEIGHT_MAPPING_CORE_GEOMETRY_POINT_CLOUD_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace geometry {

struct Point3D {
  float x, y, z;

  Point3D() = default;
  Point3D(float x, float y, float z) : x(x), y(y), z(z) {}

  // Vector operations
  Point3D operator+(const Point3D &other) const {
    return Point3D(x + other.x, y + other.y, z + other.z);
  }
  Point3D operator-(const Point3D &other) const {
    return Point3D(x - other.x, y - other.y, z - other.z);
  }
  Point3D operator*(float scalar) const {
    return Point3D(x * scalar, y * scalar, z * scalar);
  }
  Point3D &operator+=(const Point3D &other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }

  // Utility functions
  float squaredNorm() const { return x * x + y * y + z * z; }
  float norm() const { return std::sqrt(squaredNorm()); }
  float dot(const Point3D &other) const {
    return x * other.x + y * other.y + z * other.z;
  }
  Point3D cross(const Point3D &other) const {
    return Point3D(y * other.z - z * other.y, z * other.x - x * other.z,
                   x * other.y - y * other.x);
  }
  void normalize() {
    float n = norm();
    if (n > 0) {
      x /= n;
      y /= n;
      z /= n;
    }
  }
  bool isFinite() const {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
  }
};

class PointCloud {
public:
  using Ptr = std::shared_ptr<PointCloud>;
  using ConstPtr = std::shared_ptr<const PointCloud>;
  using TimeStamp = uint64_t; // nanoseconds since epoch

  // Forward declaration
  class PointView;
  class ConstPointView;
  class iterator;
  class const_iterator;

  // Constructor
  PointCloud() = default;
  explicit PointCloud(size_t reserve_size) { points_.reserve(reserve_size); }

  // PointView - Lightweight proxy for unified point + sensor data access
  class PointView {
  public:
    PointView(PointCloud *cloud, size_t index) : cloud_(cloud), index_(index) {}

    // Geometry access (mutable)
    float &x() { return cloud_->points_[index_].x; }
    float &y() { return cloud_->points_[index_].y; }
    float &z() { return cloud_->points_[index_].z; }
    Point3D &xyz() { return cloud_->points_[index_]; }

    // Read-only access (for const PointView)
    float x() const { return cloud_->points_[index_].x; }
    float y() const { return cloud_->points_[index_].y; }
    float z() const { return cloud_->points_[index_].z; }
    const Point3D &xyz() const { return cloud_->points_[index_]; }

    // Sensor data access (safe with defaults)
    float intensity() const {
      return cloud_->hasIntensity() ? (*cloud_->intensity_)[index_] : 0.0f;
    }
    void setIntensity(float val) {
      if (cloud_->hasIntensity())
        (*cloud_->intensity_)[index_] = val;
    }

    std::array<uint8_t, 3> color() const {
      return cloud_->hasColor() ? (*cloud_->color_)[index_]
                                : std::array<uint8_t, 3>{0, 0, 0};
    }
    void setColor(const std::array<uint8_t, 3> &val) {
      if (cloud_->hasColor())
        (*cloud_->color_)[index_] = val;
    }

    // Query sensor availability
    bool hasIntensity() const { return cloud_->hasIntensity(); }
    bool hasColor() const { return cloud_->hasColor(); }

    // Using Point3D Utility
    float norm() const { return cloud_->points_[index_].norm(); }
    bool isFinite() const { return cloud_->points_[index_].isFinite(); }

  private:
    PointCloud *cloud_;
    size_t index_;
  };

  class ConstPointView {
  public:
    ConstPointView(const PointCloud *cloud, size_t index)
        : cloud_(cloud), index_(index) {}

    // Geometry access (read-only)
    float x() const { return cloud_->points_[index_].x; }
    float y() const { return cloud_->points_[index_].y; }
    float z() const { return cloud_->points_[index_].z; }
    const Point3D &xyz() const { return cloud_->points_[index_]; }

    // Sensor data access (safe with defaults)
    float intensity() const {
      return cloud_->hasIntensity() ? (*cloud_->intensity_)[index_] : 0.0f;
    }
    std::array<uint8_t, 3> color() const {
      return cloud_->hasColor() ? (*cloud_->color_)[index_]
                                : std::array<uint8_t, 3>{0, 0, 0};
    }

    // Query sensor availability
    bool hasIntensity() const { return cloud_->hasIntensity(); }
    bool hasColor() const { return cloud_->hasColor(); }

    // Utility
    float norm() const { return cloud_->points_[index_].norm(); }
    bool isFinite() const { return cloud_->points_[index_].isFinite(); }

  private:
    const PointCloud *cloud_;
    size_t index_;
  };

  // Size and capacity
  size_t size() const { return points_.size(); }
  bool empty() const { return points_.empty(); }
  void reserve(size_t n) {
    points_.reserve(n);
    if (intensity_)
      intensity_->reserve(n);
    if (color_)
      color_->reserve(n);
  }
  void resize(size_t n) {
    points_.resize(n);
    if (intensity_)
      intensity_->resize(n);
    if (color_)
      color_->resize(n);
  }
  void clear() {
    points_.clear();
    if (intensity_)
      intensity_->clear();
    if (color_)
      color_->clear();
  }

  // Element access - PointView for unified access
  PointView operator[](size_t i) { return PointView(this, i); }
  ConstPointView operator[](size_t i) const { return ConstPointView(this, i); }

  PointView at(size_t i) { return PointView(this, i); }
  ConstPointView at(size_t i) const { return ConstPointView(this, i); }

  // Metadata access
  TimeStamp timestamp() const { return timestamp_ns_; }
  void setTimestamp(TimeStamp ts) { timestamp_ns_ = ts; }
  const std::string &frameId() const { return frame_id_; }
  void setFrameId(const std::string &id) { frame_id_ = id; }

  // Custom iterator that returns PointViews for consistent API
  class iterator {
  public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = PointView;
    using difference_type = std::ptrdiff_t;
    using pointer = PointView *;
    using reference = PointView;

    iterator(PointCloud *cloud, size_t index) : cloud_(cloud), index_(index) {}

    PointView operator*() const { return PointView(cloud_, index_); }
    iterator &operator++() {
      ++index_;
      return *this;
    }
    iterator operator++(int) {
      iterator tmp = *this;
      ++index_;
      return tmp;
    }
    bool operator==(const iterator &other) const {
      return index_ == other.index_;
    }
    bool operator!=(const iterator &other) const {
      return index_ != other.index_;
    }

  private:
    PointCloud *cloud_;
    size_t index_;
  };

  class const_iterator {
  public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = ConstPointView;
    using difference_type = std::ptrdiff_t;
    using pointer = ConstPointView *;
    using reference = ConstPointView;

    const_iterator(const PointCloud *cloud, size_t index)
        : cloud_(cloud), index_(index) {}

    ConstPointView operator*() const { return ConstPointView(cloud_, index_); }
    const_iterator &operator++() {
      ++index_;
      return *this;
    }
    const_iterator operator++(int) {
      const_iterator tmp = *this;
      ++index_;
      return tmp;
    }
    bool operator==(const const_iterator &other) const {
      return index_ == other.index_;
    }
    bool operator!=(const const_iterator &other) const {
      return index_ != other.index_;
    }

  private:
    const PointCloud *cloud_;
    size_t index_;
  };

  // Iterators return PointViews for consistent API
  iterator begin() { return iterator(this, 0); }
  const_iterator begin() const { return const_iterator(this, 0); }
  iterator end() { return iterator(this, points_.size()); }
  const_iterator end() const { return const_iterator(this, points_.size()); }

  // Add point methods
  void push_back(const Point3D &point) {
    points_.push_back(point);
    if (intensity_)
      intensity_->push_back(0.0f); // Default intensity
    if (color_)
      color_->push_back({0, 0, 0}); // Default color
  }
  void push_back(Point3D &&point) {
    points_.push_back(std::move(point));
    if (intensity_)
      intensity_->push_back(0.0f);
    if (color_)
      color_->push_back({0, 0, 0});
  }

  template <typename... Args> void emplace_back(Args &&... args) {
    points_.emplace_back(std::forward<Args>(args)...);
    if (intensity_)
      intensity_->push_back(0.0f);
    if (color_)
      color_->push_back({0, 0, 0});
  }

  void addPoint(const Point3D &point, float intensity_val = 0.0f,
                const std::array<uint8_t, 3> &color_val = {0, 0, 0}) {
    points_.push_back(point);
    if (intensity_)
      intensity_->push_back(intensity_val);
    if (color_)
      color_->push_back(color_val);
  }

  // Concatenate clouds with automatic sensor data synchronization
  PointCloud &operator+=(const PointCloud &other) {
    // Store original size for filling defaults
    size_t original_size = points_.size();

    // Merge points
    points_.insert(points_.end(), other.points_.begin(), other.points_.end());

    // Handle intensity data
    if (hasIntensity() || other.hasIntensity()) {
      // At least one has intensity - need to merge
      if (!intensity_) {
        // Create and fill with defaults for existing points
        intensity_ = std::make_shared<std::vector<float>>(original_size, 0.0f);
      }

      if (other.hasIntensity()) {
        // Append other's intensity data
        intensity_->insert(intensity_->end(), other.intensity_->begin(),
                           other.intensity_->end());
      } else {
        // Other doesn't have intensity - fill with defaults
        intensity_->resize(points_.size(), 0.0f);
      }
    }

    // Handle color data
    if (hasColor() || other.hasColor()) {
      // At least one has color - need to merge
      if (!color_) {
        // Create and fill with defaults for existing points
        color_ = std::make_shared<std::vector<std::array<uint8_t, 3>>>(
            original_size, std::array<uint8_t, 3>{0, 0, 0});
      }

      if (other.hasColor()) {
        // Append other's color data
        color_->insert(color_->end(), other.color_->begin(),
                       other.color_->end());
      } else {
        // Other doesn't have color - fill with defaults
        color_->resize(points_.size(), std::array<uint8_t, 3>{0, 0, 0});
      }
    }

    return *this;
  }
  PointCloud operator+(const PointCloud &other) const {
    PointCloud result = *this;
    result += other;
    return result;
  }

  // Additional field management
  bool hasIntensity() const { return intensity_ && !intensity_->empty(); }
  bool hasColor() const { return color_ && !color_->empty(); }

  void attachIntensity(std::vector<float> &&intensity_data) {
    intensity_ =
        std::make_shared<std::vector<float>>(std::move(intensity_data));
  }
  void attachColor(std::vector<std::array<uint8_t, 3>> &&color_data) {
    color_ = std::make_shared<std::vector<std::array<uint8_t, 3>>>(
        std::move(color_data));
  }

  // Smart filtering with automatic sensor data synchronization
  template <typename Predicate> void filterInPlace(Predicate pred) {
    size_t write_idx = 0;
    for (size_t read_idx = 0; read_idx < points_.size(); ++read_idx) {
      if (pred(ConstPointView(this, read_idx))) {
        // Move point
        if (write_idx != read_idx) {
          points_[write_idx] = points_[read_idx];
          // Move sensor data if present
          if (hasIntensity())
            (*intensity_)[write_idx] = (*intensity_)[read_idx];
          if (hasColor())
            (*color_)[write_idx] = (*color_)[read_idx];
        }
        write_idx++;
      }
    }
    // Resize all arrays
    points_.resize(write_idx);
    if (hasIntensity())
      intensity_->resize(write_idx);
    if (hasColor())
      color_->resize(write_idx);
  }

  // Swap clouds
  void swap(PointCloud &other) {
    std::swap(points_, other.points_);
    std::swap(frame_id_, other.frame_id_);
    std::swap(timestamp_ns_, other.timestamp_ns_);
    std::swap(intensity_, other.intensity_);
    std::swap(color_, other.color_);
  }

private:
  std::vector<Point3D> points_;
  TimeStamp timestamp_ns_{0}; // nanoseconds since epoch
  std::string frame_id_;
  std::shared_ptr<std::vector<float>> intensity_;              // Optional
  std::shared_ptr<std::vector<std::array<uint8_t, 3>>> color_; // Optional
};

} // namespace geometry

#endif // HEIGHT_MAPPING_CORE_GEOMETRY_POINT_CLOUD_H