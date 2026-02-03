// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_CORE_IMPL_POINT_CLOUD_IMPL_HPP
#define NANOPCL_CORE_IMPL_POINT_CLOUD_IMPL_HPP

namespace nanopcl {

inline void PointCloud::resize(size_t n) {
  points_.resize(n, Point4(0, 0, 0, 1));
  syncChannelSizes(n);
}

inline void PointCloud::reserve(size_t n) {
  points_.reserve(n);
  if (use_intensity_) intensity_.reserve(n);
  if (use_time_) time_.reserve(n);
  if (use_ring_) ring_.reserve(n);
  if (use_color_) color_.reserve(n);
  if (use_label_) label_.reserve(n);
  if (use_normal_) normal_.reserve(n);
  if (use_covariance_) covariance_.reserve(n);
}

inline void PointCloud::clear() {
  points_.clear();
  if (use_intensity_) intensity_.clear();
  if (use_time_) time_.clear();
  if (use_ring_) ring_.clear();
  if (use_color_) color_.clear();
  if (use_label_) label_.clear();
  if (use_normal_) normal_.clear();
  if (use_covariance_) covariance_.clear();
}

inline void PointCloud::reset() {
  points_.clear();
  points_.shrink_to_fit();

  intensity_.clear();
  intensity_.shrink_to_fit();
  time_.clear();
  time_.shrink_to_fit();
  ring_.clear();
  ring_.shrink_to_fit();
  color_.clear();
  color_.shrink_to_fit();
  label_.clear();
  label_.shrink_to_fit();
  normal_.clear();
  normal_.shrink_to_fit();
  covariance_.clear();
  covariance_.shrink_to_fit();

  use_intensity_ = false;
  use_time_ = false;
  use_ring_ = false;
  use_color_ = false;
  use_label_ = false;
  use_normal_ = false;
  use_covariance_ = false;

  frame_id_.clear();
  timestamp_ns_ = 0;
}

inline void PointCloud::useIntensity() {
  use_intensity_ = true;
  if (intensity_.empty() && !points_.empty()) {
    intensity_.resize(points_.size(), 0.0f);
  }
}

inline void PointCloud::useTime() {
  use_time_ = true;
  if (time_.empty() && !points_.empty()) {
    time_.resize(points_.size(), 0.0f);
  }
}

inline void PointCloud::useRing() {
  use_ring_ = true;
  if (ring_.empty() && !points_.empty()) {
    ring_.resize(points_.size(), 0);
  }
}

inline void PointCloud::useColor() {
  use_color_ = true;
  if (color_.empty() && !points_.empty()) {
    color_.resize(points_.size(), Color());
  }
}

inline void PointCloud::useLabel() {
  use_label_ = true;
  if (label_.empty() && !points_.empty()) {
    label_.resize(points_.size(), Label());
  }
}

inline void PointCloud::useNormal() {
  use_normal_ = true;
  if (normal_.empty() && !points_.empty()) {
    normal_.resize(points_.size(), Normal4(0, 0, 0, 0));
  }
}

inline void PointCloud::useCovariance() {
  use_covariance_ = true;
  if (covariance_.empty() && !points_.empty()) {
    covariance_.resize(points_.size(), Covariance::Zero());
  }
}

inline void PointCloud::add(float x, float y, float z) {
  points_.push_back(Point4(x, y, z, 1));
  pushDefaultChannelValues();
}

inline void PointCloud::copyChannelLayout(const PointCloud& other) {
  if (other.hasIntensity()) useIntensity();
  if (other.hasTime()) useTime();
  if (other.hasRing()) useRing();
  if (other.hasColor()) useColor();
  if (other.hasLabel()) useLabel();
  if (other.hasNormal()) useNormal();
  if (other.hasCovariance()) useCovariance();
}

inline void PointCloud::copyChannelData(size_t dst_idx, const PointCloud& src, size_t src_idx) {
  if (use_intensity_ && src.hasIntensity())
    intensity_[dst_idx] = src.intensity_[src_idx];
  if (use_time_ && src.hasTime()) time_[dst_idx] = src.time_[src_idx];
  if (use_ring_ && src.hasRing()) ring_[dst_idx] = src.ring_[src_idx];
  if (use_color_ && src.hasColor()) color_[dst_idx] = src.color_[src_idx];
  if (use_label_ && src.hasLabel()) label_[dst_idx] = src.label_[src_idx];
  if (use_normal_ && src.hasNormal()) normal_[dst_idx] = src.normal_[src_idx];
  if (use_covariance_ && src.hasCovariance())
    covariance_[dst_idx] = src.covariance_[src_idx];
}

inline void PointCloud::appendChannelData(const PointCloud& src) {
  if (use_intensity_ && src.hasIntensity())
    intensity_.insert(intensity_.end(), src.intensity_.begin(), src.intensity_.end());
  if (use_time_ && src.hasTime())
    time_.insert(time_.end(), src.time_.begin(), src.time_.end());
  if (use_ring_ && src.hasRing())
    ring_.insert(ring_.end(), src.ring_.begin(), src.ring_.end());
  if (use_color_ && src.hasColor())
    color_.insert(color_.end(), src.color_.begin(), src.color_.end());
  if (use_label_ && src.hasLabel())
    label_.insert(label_.end(), src.label_.begin(), src.label_.end());
  if (use_normal_ && src.hasNormal())
    normal_.insert(normal_.end(), src.normal_.begin(), src.normal_.end());
  if (use_covariance_ && src.hasCovariance())
    covariance_.insert(covariance_.end(), src.covariance_.begin(), src.covariance_.end());
}

inline PointCloud& PointCloud::operator+=(const PointCloud& other) {
  if (other.empty()) return *this;

  if (empty()) {
    copyChannelLayout(other);
  }

  size_t new_size = size() + other.size();
  reserve(new_size);

  points_.insert(points_.end(), other.points_.begin(), other.points_.end());
  appendChannelData(other);
  syncChannelSizes(new_size);

  return *this;
}

inline PointCloud PointCloud::extract(
    const std::vector<size_t>& indices) const {
  PointCloud result;
  result.copyChannelLayout(*this);
  result.reserve(indices.size());

  for (size_t idx : indices)
    result.points_.push_back(points_[idx]);
  if (use_intensity_) {
    for (size_t idx : indices)
      result.intensity_.push_back(intensity_[idx]);
  }
  if (use_time_) {
    for (size_t idx : indices)
      result.time_.push_back(time_[idx]);
  }
  if (use_ring_) {
    for (size_t idx : indices)
      result.ring_.push_back(ring_[idx]);
  }
  if (use_color_) {
    for (size_t idx : indices)
      result.color_.push_back(color_[idx]);
  }
  if (use_label_) {
    for (size_t idx : indices)
      result.label_.push_back(label_[idx]);
  }
  if (use_normal_) {
    for (size_t idx : indices)
      result.normal_.push_back(normal_[idx]);
  }
  if (use_covariance_) {
    for (size_t idx : indices)
      result.covariance_.push_back(covariance_[idx]);
  }

  result.frame_id_ = frame_id_;
  result.timestamp_ns_ = timestamp_ns_;
  return result;
}

inline PointCloud PointCloud::extract(size_t start, size_t count) const {
  PointCloud result;
  result.copyChannelLayout(*this);
  result.resize(count);

  std::copy_n(points_.begin() + start, count, result.points_.begin());
  if (use_intensity_)
    std::copy_n(intensity_.begin() + start, count, result.intensity_.begin());
  if (use_time_)
    std::copy_n(time_.begin() + start, count, result.time_.begin());
  if (use_ring_)
    std::copy_n(ring_.begin() + start, count, result.ring_.begin());
  if (use_color_)
    std::copy_n(color_.begin() + start, count, result.color_.begin());
  if (use_label_)
    std::copy_n(label_.begin() + start, count, result.label_.begin());
  if (use_normal_)
    std::copy_n(normal_.begin() + start, count, result.normal_.begin());
  if (use_covariance_)
    std::copy_n(covariance_.begin() + start, count, result.covariance_.begin());

  result.frame_id_ = frame_id_;
  result.timestamp_ns_ = timestamp_ns_;
  return result;
}

inline void PointCloud::erase(const std::vector<size_t>& indices) {
  if (indices.empty()) return;

  std::vector<size_t> sorted = indices;
  std::sort(sorted.begin(), sorted.end());
  sorted.erase(std::unique(sorted.begin(), sorted.end()), sorted.end());

  const size_t n = size();
  const size_t del_n = sorted.size();

  // Cache pointers to avoid repeated member access
  Point4* p_ptr = points_.data();
  float* i_ptr = use_intensity_ ? intensity_.data() : nullptr;
  float* t_ptr = use_time_ ? time_.data() : nullptr;
  uint16_t* r_ptr = use_ring_ ? ring_.data() : nullptr;
  Color* c_ptr = use_color_ ? color_.data() : nullptr;
  Label* l_ptr = use_label_ ? label_.data() : nullptr;
  Normal4* n_ptr = use_normal_ ? normal_.data() : nullptr;
  Covariance* cov_ptr = use_covariance_ ? covariance_.data() : nullptr;

  size_t write = 0;
  size_t idx_pos = 0;
  for (size_t read = 0; read < n; ++read) {
    if (idx_pos < del_n && read == sorted[idx_pos]) {
      ++idx_pos;
      continue;
    }
    if (write != read) {
      p_ptr[write] = std::move(p_ptr[read]);
      if (i_ptr) i_ptr[write] = i_ptr[read];
      if (t_ptr) t_ptr[write] = t_ptr[read];
      if (r_ptr) r_ptr[write] = r_ptr[read];
      if (c_ptr) c_ptr[write] = c_ptr[read];
      if (l_ptr) l_ptr[write] = l_ptr[read];
      if (n_ptr) n_ptr[write] = std::move(n_ptr[read]);
      if (cov_ptr) cov_ptr[write] = std::move(cov_ptr[read]);
    }
    ++write;
  }

  resize(write);
}

inline void PointCloud::pushDefaultChannelValues() {
  if (use_intensity_) intensity_.push_back(0.0f);
  if (use_time_) time_.push_back(0.0f);
  if (use_ring_) ring_.push_back(0);
  if (use_color_) color_.push_back(Color());
  if (use_label_) label_.push_back(Label());
  if (use_normal_) normal_.push_back(Normal4(0, 0, 0, 0));
  if (use_covariance_) covariance_.push_back(Covariance::Zero());
}

inline void PointCloud::syncChannelSizes(size_t n) {
  if (use_intensity_) intensity_.resize(n, 0.0f);
  if (use_time_) time_.resize(n, 0.0f);
  if (use_ring_) ring_.resize(n, 0);
  if (use_color_) color_.resize(n, Color());
  if (use_label_) label_.resize(n, Label());
  if (use_normal_) normal_.resize(n, Normal4(0, 0, 0, 0));
  if (use_covariance_) covariance_.resize(n, Covariance::Zero());
}

} // namespace nanopcl

#endif // NANOPCL_CORE_IMPL_POINT_CLOUD_IMPL_HPP
