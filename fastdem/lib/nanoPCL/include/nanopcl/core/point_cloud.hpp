// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_CORE_POINT_CLOUD_HPP
#define NANOPCL_CORE_POINT_CLOUD_HPP

#include <algorithm>
#include <string>

#include "nanopcl/core/types.hpp"

namespace nanopcl {

class PointCloud {
public:
  // Constructors
  PointCloud() = default;
  explicit PointCloud(size_t n) { resize(n); }

  // Size & Capacity
  size_t size() const { return points_.size(); }
  bool empty() const { return points_.empty(); }
  size_t capacity() const { return points_.capacity(); }
  IndexRange indices() const { return IndexRange(size()); }

  void resize(size_t n);
  void reserve(size_t n);
  void clear(); // Data only, keep channel structure
  void reset(); // Full reset including channels

  // Point Access (Safe - 3D)
  auto point(size_t i) { return points_[i].head<3>(); }
  auto point(size_t i) const { return points_[i].head<3>(); }

  // Point Access (Expert - 4D for transforms)
  Point4& operator[](size_t i) { return points_[i]; }
  const Point4& operator[](size_t i) const { return points_[i]; }

  AlignedVector<Point4>& points() { return points_; }
  const AlignedVector<Point4>& points() const { return points_; }

  // Point Addition
  void add(float x, float y, float z);

  template <typename... Attrs>
  void add(float x, float y, float z, Attrs&&... attrs) {
    points_.push_back(Point4(x, y, z, 1));
    pushDefaultChannelValues();
    (applyAttr(std::forward<Attrs>(attrs)), ...);
  }

  // Channels - Intensity
  bool hasIntensity() const { return use_intensity_; }
  void useIntensity();
  std::vector<float>& intensities() { return intensity_; }
  const std::vector<float>& intensities() const { return intensity_; }
  float& intensity(size_t i) { return intensity_[i]; }
  float intensity(size_t i) const { return intensity_[i]; }

  // Channels - Time
  bool hasTime() const { return use_time_; }
  void useTime();
  std::vector<float>& times() { return time_; }
  const std::vector<float>& times() const { return time_; }
  float& time(size_t i) { return time_[i]; }
  float time(size_t i) const { return time_[i]; }

  // Channels - Ring
  bool hasRing() const { return use_ring_; }
  void useRing();
  std::vector<uint16_t>& rings() { return ring_; }
  const std::vector<uint16_t>& rings() const { return ring_; }
  uint16_t& ring(size_t i) { return ring_[i]; }
  uint16_t ring(size_t i) const { return ring_[i]; }

  // Channels - Color
  bool hasColor() const { return use_color_; }
  void useColor();
  std::vector<Color>& colors() { return color_; }
  const std::vector<Color>& colors() const { return color_; }
  Color& color(size_t i) { return color_[i]; }
  const Color& color(size_t i) const { return color_[i]; }

  // Channels - Label
  bool hasLabel() const { return use_label_; }
  void useLabel();
  std::vector<Label>& labels() { return label_; }
  const std::vector<Label>& labels() const { return label_; }
  Label& label(size_t i) { return label_[i]; }
  const Label& label(size_t i) const { return label_[i]; }

  // Channels - Normal (Safe - 3D)
  bool hasNormal() const { return use_normal_; }
  void useNormal();
  AlignedVector<Normal4>& normals() { return normal_; }
  const AlignedVector<Normal4>& normals() const { return normal_; }
  auto normal(size_t i) { return normal_[i].head<3>(); }
  auto normal(size_t i) const { return normal_[i].head<3>(); }

  // Channels - Covariance
  bool hasCovariance() const { return use_covariance_; }
  void useCovariance();
  AlignedVector<Covariance>& covariances() { return covariance_; }
  const AlignedVector<Covariance>& covariances() const { return covariance_; }
  Covariance& covariance(size_t i) { return covariance_[i]; }
  const Covariance& covariance(size_t i) const { return covariance_[i]; }

  // Channel Utilities
  void copyChannelLayout(const PointCloud& other);
  void copyChannelData(size_t dst_idx, const PointCloud& src, size_t src_idx);
  void appendChannelData(const PointCloud& src);

  // Metadata
  const std::string& frameId() const { return frame_id_; }
  void setFrameId(const std::string& id) { frame_id_ = id; }
  uint64_t timestamp() const { return timestamp_ns_; }
  void setTimestamp(uint64_t ns) { timestamp_ns_ = ns; }

  // Operations
  PointCloud& operator+=(const PointCloud& other);
  PointCloud extract(const std::vector<size_t>& indices) const;
  PointCloud extract(size_t start, size_t count) const;
  void erase(const std::vector<size_t>& indices);

private:
  // Data
  AlignedVector<Point4> points_;
  std::vector<float> intensity_;
  std::vector<float> time_;
  std::vector<uint16_t> ring_;
  std::vector<Color> color_;
  std::vector<Label> label_;
  AlignedVector<Normal4> normal_;
  AlignedVector<Covariance> covariance_;

  // Metadata
  std::string frame_id_;
  uint64_t timestamp_ns_ = 0;

  // Channel flags
  bool use_intensity_ = false;
  bool use_time_ = false;
  bool use_ring_ = false;
  bool use_color_ = false;
  bool use_label_ = false;
  bool use_normal_ = false;
  bool use_covariance_ = false;

  // Internal helpers
  void pushDefaultChannelValues();
  void syncChannelSizes(size_t n);

  // Variadic add helpers
  void applyAttr(Intensity a) {
    if (!use_intensity_)
      useIntensity();
    intensity_.back() = a;
  }
  void applyAttr(Time a) {
    if (!use_time_)
      useTime();
    time_.back() = a;
  }
  void applyAttr(Ring a) {
    if (!use_ring_)
      useRing();
    ring_.back() = a;
  }
  void applyAttr(const Color& a) {
    if (!use_color_)
      useColor();
    color_.back() = a;
  }
  void applyAttr(Label a) {
    if (!use_label_)
      useLabel();
    label_.back() = a;
  }
  void applyAttr(const Eigen::Vector3f& n) {
    if (!use_normal_)
      useNormal();
    normal_.back().head<3>() = n;
  }
};

} // namespace nanopcl

#include "nanopcl/core/impl/point_cloud_impl.hpp"

#endif // NANOPCL_CORE_POINT_CLOUD_HPP
