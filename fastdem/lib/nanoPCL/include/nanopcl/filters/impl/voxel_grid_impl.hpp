// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_FILTERS_IMPL_VOXEL_GRID_IMPL_HPP
#define NANOPCL_FILTERS_IMPL_VOXEL_GRID_IMPL_HPP

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "nanopcl/core/voxel.hpp"

namespace nanopcl {
namespace filters {
namespace detail {

inline std::vector<voxel::IndexedPoint>& getIndexBuffer() {
  thread_local std::vector<voxel::IndexedPoint> buffer;
  return buffer;
}

} // namespace detail

// =============================================================================
// Voxel Grid (Move version) - Main implementation
// =============================================================================

inline PointCloud voxelGrid(PointCloud&& cloud, float voxel_size, VoxelMode mode) {
  if (voxel_size < voxel::MIN_SIZE || voxel_size > voxel::MAX_SIZE) {
    throw std::invalid_argument("voxel_size must be in [0.001, 100]");
  }
  if (cloud.empty())
    return std::move(cloud);

  const bool has_i = cloud.hasIntensity();
  const bool has_t = cloud.hasTime();
  const bool has_r = cloud.hasRing();
  const bool has_c = cloud.hasColor();
  const bool has_l = cloud.hasLabel();
  const bool has_n = cloud.hasNormal();
  const bool has_cov = cloud.hasCovariance();

  // Step 1: Compute voxel keys
  const float inv = 1.0f / voxel_size;
  auto& idx = detail::getIndexBuffer();
  idx.clear();
  idx.reserve(cloud.size());

  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto& p = cloud[i];
    if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z()))
      continue;
    idx.push_back({voxel::pack(p, inv), static_cast<uint32_t>(i)});
  }
  if (idx.empty()) {
    cloud.clear();
    return std::move(cloud);
  }

  // Step 2: Sort by voxel key
  std::sort(idx.begin(), idx.end());

  // Step 3: Count unique voxels for memory reservation
  size_t num_voxels = 0;
  {
    size_t i = 0;
    while (i < idx.size()) {
      uint64_t key = idx[i].key;
      while (i < idx.size() && idx[i].key == key)
        ++i;
      ++num_voxels;
    }
  }

  // Step 4: Create result cloud
  PointCloud result;
  result.copyChannelLayout(cloud);
  result.reserve(num_voxels);
  result.setFrameId(cloud.frameId());
  result.setTimestamp(cloud.timestamp());

  // Step 5: Process voxels and build result
  size_t start = 0;

  while (start < idx.size()) {
    uint64_t key = idx[start].key;
    size_t end = start + 1;
    while (end < idx.size() && idx[end].key == key)
      ++end;

    const size_t count = end - start;
    const float n = static_cast<float>(count);
    uint32_t rep = idx[start].index;

    switch (mode) {
    case VoxelMode::CENTROID: {
      Point4 sum = Point4::Zero();
      float si = 0, st = 0, sr = 0, sg = 0, sb = 0;
      Eigen::Vector3f sn = Eigen::Vector3f::Zero();
      for (size_t j = start; j < end; ++j) {
        uint32_t i = idx[j].index;
        sum += cloud[i];
        if (has_i)
          si += cloud.intensity(i);
        if (has_t)
          st += cloud.time(i);
        if (has_c) {
          sr += cloud.color(i).r;
          sg += cloud.color(i).g;
          sb += cloud.color(i).b;
        }
        if (has_n)
          sn += cloud.normals()[i].head<3>();
      }
      Point4 pt = sum / n;
      pt.w() = 1.0f;
      result.points().push_back(pt);
      if (has_i)
        result.intensities().push_back(si / n);
      if (has_t)
        result.times().push_back(st / n);
      if (has_r)
        result.rings().push_back(cloud.ring(rep));
      if (has_c)
        result.colors().push_back(Color(static_cast<uint8_t>(sr / n),
                                        static_cast<uint8_t>(sg / n),
                                        static_cast<uint8_t>(sb / n)));
      if (has_l)
        result.labels().push_back(cloud.label(rep));
      if (has_n) {
        float norm = sn.norm();
        Eigen::Vector3f n3 = (norm > 1e-6f) ? (sn / norm).eval() : Eigen::Vector3f::UnitZ();
        result.normals().push_back(Normal4(n3.x(), n3.y(), n3.z(), 0));
      }
      if (has_cov)
        result.covariances().push_back(cloud.covariance(rep));
      break;
    }

    case VoxelMode::NEAREST: {
      Point4 center = voxel::toCenter(key, voxel_size);
      float min_d2 = std::numeric_limits<float>::max();
      uint32_t nearest = rep;
      for (size_t j = start; j < end; ++j) {
        float d2 = (cloud[idx[j].index] - center).squaredNorm();
        if (d2 < min_d2) {
          min_d2 = d2;
          nearest = idx[j].index;
        }
      }
      result.points().push_back(cloud[nearest]);
      if (has_i)
        result.intensities().push_back(cloud.intensity(nearest));
      if (has_t)
        result.times().push_back(cloud.time(nearest));
      if (has_r)
        result.rings().push_back(cloud.ring(nearest));
      if (has_c)
        result.colors().push_back(cloud.color(nearest));
      if (has_l)
        result.labels().push_back(cloud.label(nearest));
      if (has_n)
        result.normals().push_back(cloud.normals()[nearest]);
      if (has_cov)
        result.covariances().push_back(cloud.covariance(nearest));
      break;
    }

    case VoxelMode::ANY: {
      uint32_t sel = idx[start + (count * 7 + start * 13) % count].index;
      result.points().push_back(cloud[sel]);
      if (has_i)
        result.intensities().push_back(cloud.intensity(sel));
      if (has_t)
        result.times().push_back(cloud.time(sel));
      if (has_r)
        result.rings().push_back(cloud.ring(sel));
      if (has_c)
        result.colors().push_back(cloud.color(sel));
      if (has_l)
        result.labels().push_back(cloud.label(sel));
      if (has_n)
        result.normals().push_back(cloud.normals()[sel]);
      if (has_cov)
        result.covariances().push_back(cloud.covariance(sel));
      break;
    }

    case VoxelMode::CENTER: {
      float si = 0, st = 0, sr = 0, sg = 0, sb = 0;
      Eigen::Vector3f sn = Eigen::Vector3f::Zero();
      for (size_t j = start; j < end; ++j) {
        uint32_t i = idx[j].index;
        if (has_i)
          si += cloud.intensity(i);
        if (has_t)
          st += cloud.time(i);
        if (has_c) {
          sr += cloud.color(i).r;
          sg += cloud.color(i).g;
          sb += cloud.color(i).b;
        }
        if (has_n)
          sn += cloud.normals()[i].head<3>();
      }
      result.points().push_back(voxel::toCenter(key, voxel_size));
      if (has_i)
        result.intensities().push_back(si / n);
      if (has_t)
        result.times().push_back(st / n);
      if (has_r)
        result.rings().push_back(cloud.ring(rep));
      if (has_c)
        result.colors().push_back(Color(static_cast<uint8_t>(sr / n),
                                        static_cast<uint8_t>(sg / n),
                                        static_cast<uint8_t>(sb / n)));
      if (has_l)
        result.labels().push_back(cloud.label(rep));
      if (has_n) {
        float norm = sn.norm();
        Eigen::Vector3f n3 = (norm > 1e-6f) ? (sn / norm).eval() : Eigen::Vector3f::UnitZ();
        result.normals().push_back(Normal4(n3.x(), n3.y(), n3.z(), 0));
      }
      if (has_cov)
        result.covariances().push_back(cloud.covariance(rep));
      break;
    }
    }

    start = end;
  }

  return std::move(result);
}

// =============================================================================
// Voxel Grid (Copy version) - Delegates to Move version
// =============================================================================

inline PointCloud voxelGrid(const PointCloud& cloud, float voxel_size, VoxelMode mode) {
  PointCloud copy = cloud;
  return voxelGrid(std::move(copy), voxel_size, mode);
}

} // namespace filters
} // namespace nanopcl

#endif // NANOPCL_FILTERS_IMPL_VOXEL_GRID_IMPL_HPP
