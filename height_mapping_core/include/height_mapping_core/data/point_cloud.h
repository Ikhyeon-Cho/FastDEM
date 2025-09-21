/*
 * point_cloud.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_DATA_POINT_CLOUD_H
#define HEIGHT_MAPPING_CORE_DATA_POINT_CLOUD_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace height_map {

// Basic 3D point
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

// Point with intensity (for LiDAR)
struct PointXYZI : public Point3D {
  float intensity;

  PointXYZI() = default;
  PointXYZI(float x, float y, float z, float intensity = 0.0f)
      : Point3D(x, y, z), intensity(intensity) {}
};

// Point with RGB color (for RGB-D cameras)
struct PointXYZRGB : public Point3D {
  uint8_t r, g, b;

  PointXYZRGB() = default;
  PointXYZRGB(float x, float y, float z, uint8_t r = 0, uint8_t g = 0,
              uint8_t b = 0)
      : Point3D(x, y, z), r(r), g(g), b(b) {}

  uint32_t rgb() const {
    return (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) |
           static_cast<uint32_t>(b);
  }

  void setRGB(uint32_t rgb) {
    r = (rgb >> 16) & 0xFF;
    g = (rgb >> 8) & 0xFF;
    b = rgb & 0xFF;
  }
};

// Generic point cloud container
template <typename PointT = Point3D> class PointCloud {
public:
  using PointType = PointT;
  using Ptr = std::shared_ptr<PointCloud<PointT>>;
  using ConstPtr = std::shared_ptr<const PointCloud<PointT>>;
  using TimeStamp = uint64_t; // nanoseconds since epoch

  // Data members
  std::vector<PointT> points;
  TimeStamp timestamp = 0;
  std::string frame_id;

  // Sensor origin (optional, for ray tracing)
  Point3D sensor_origin = {0, 0, 0};

  // Constructor
  PointCloud() = default;

  explicit PointCloud(size_t reserve_size) { points.reserve(reserve_size); }

  // Factory methods removed - use std::make_shared directly for consistency

  // Size and capacity
  size_t size() const { return points.size(); }
  bool empty() const { return points.empty(); }
  void reserve(size_t n) { points.reserve(n); }
  void resize(size_t n) { points.resize(n); }
  void clear() { points.clear(); }

  // Element access
  PointT &operator[](size_t i) { return points[i]; }
  const PointT &operator[](size_t i) const { return points[i]; }

  PointT &at(size_t i) { return points.at(i); }
  const PointT &at(size_t i) const { return points.at(i); }

  PointT &front() { return points.front(); }
  const PointT &front() const { return points.front(); }

  PointT &back() { return points.back(); }
  const PointT &back() const { return points.back(); }

  // Iterators
  auto begin() { return points.begin(); }
  auto begin() const { return points.begin(); }
  auto end() { return points.end(); }
  auto end() const { return points.end(); }

  // Add points
  void push_back(const PointT &point) { points.push_back(point); }

  void push_back(PointT &&point) { points.push_back(std::move(point)); }

  template <typename... Args> void emplace_back(Args &&... args) {
    points.emplace_back(std::forward<Args>(args)...);
  }

  // Concatenate clouds
  PointCloud &operator+=(const PointCloud &other) {
    points.insert(points.end(), other.points.begin(), other.points.end());
    return *this;
  }

  PointCloud operator+(const PointCloud &other) const {
    PointCloud result = *this;
    result += other;
    return result;
  }

  // Filter operations
  void removeNaN() {
    points.erase(
        std::remove_if(points.begin(), points.end(),
                       [](const PointT &p) {
                         if constexpr (std::is_base_of_v<Point3D, PointT>) {
                           return !p.isFinite();
                         } else {
                           return !std::isfinite(p.x) || !std::isfinite(p.y) ||
                                  !std::isfinite(p.z);
                         }
                       }),
        points.end());
  }

  template <typename Predicate> void filterInPlace(Predicate pred) {
    points.erase(std::remove_if(points.begin(), points.end(),
                                [&pred](const PointT &p) { return !pred(p); }),
                 points.end());
  }

  template <typename Predicate> PointCloud filter(Predicate pred) const {
    PointCloud result;
    result.reserve(points.size() / 2); // Estimate

    for (const auto &point : points) {
      if (pred(point)) {
        result.push_back(point);
      }
    }

    result.timestamp = timestamp;
    result.frame_id = frame_id;
    result.sensor_origin = sensor_origin;

    return result;
  }

  // Transform operations
  template <typename Transform>
  void transformInPlace(const Transform &transform) {
    for (auto &point : points) {
      point = transform.apply(point);
    }
    sensor_origin = transform.apply(sensor_origin);
  }

  template <typename Transform>
  PointCloud transform(const Transform &transform) const {
    PointCloud result = *this;
    result.transformInPlace(transform);
    return result;
  }

  // Bounding box
  struct BoundingBox {
    Point3D min{std::numeric_limits<float>::max(),
                std::numeric_limits<float>::max(),
                std::numeric_limits<float>::max()};
    Point3D max{std::numeric_limits<float>::lowest(),
                std::numeric_limits<float>::lowest(),
                std::numeric_limits<float>::lowest()};

    Point3D center() const {
      return Point3D((min.x + max.x) / 2, (min.y + max.y) / 2,
                     (min.z + max.z) / 2);
    }

    Point3D size() const {
      return Point3D(max.x - min.x, max.y - min.y, max.z - min.z);
    }

    bool contains(const Point3D &p) const {
      return p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y &&
             p.z >= min.z && p.z <= max.z;
    }
  };

  BoundingBox computeBoundingBox() const {
    BoundingBox bbox;

    for (const auto &point : points) {
      if constexpr (std::is_base_of_v<Point3D, PointT>) {
        if (!point.isFinite())
          continue;
      } else {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z))
          continue;
      }

      bbox.min.x = std::min(bbox.min.x, point.x);
      bbox.min.y = std::min(bbox.min.y, point.y);
      bbox.min.z = std::min(bbox.min.z, point.z);
      bbox.max.x = std::max(bbox.max.x, point.x);
      bbox.max.y = std::max(bbox.max.y, point.y);
      bbox.max.z = std::max(bbox.max.z, point.z);
    }

    return bbox;
  }

  // Statistics
  Point3D computeCentroid() const {
    if (points.empty())
      return Point3D(0, 0, 0);

    Point3D centroid(0, 0, 0);
    size_t valid_count = 0;

    for (const auto &point : points) {
      if constexpr (std::is_base_of_v<Point3D, PointT>) {
        if (!point.isFinite())
          continue;
      } else {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z))
          continue;
      }

      centroid.x += point.x;
      centroid.y += point.y;
      centroid.z += point.z;
      valid_count++;
    }

    if (valid_count > 0) {
      centroid.x /= valid_count;
      centroid.y /= valid_count;
      centroid.z /= valid_count;
    }

    return centroid;
  }

  // Swap contents
  void swap(PointCloud &other) {
    points.swap(other.points);
    std::swap(timestamp, other.timestamp);
    frame_id.swap(other.frame_id);
    std::swap(sensor_origin, other.sensor_origin);
  }
};

// Type aliases for common point cloud types
using PointCloudXYZ = PointCloud<Point3D>;
using PointCloudXYZI = PointCloud<PointXYZI>;
using PointCloudXYZRGB = PointCloud<PointXYZRGB>;

} // namespace height_map

#endif // HEIGHT_MAPPING_CORE_DATA_POINT_CLOUD_H