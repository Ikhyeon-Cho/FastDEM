// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>
//
// Benchmark: postprocess iteration patterns
//
// Compares manual lr/lc circular buffer wrapping (current) vs:
//   A) GridMapIterator + CircleIterator
//   B) GridMapIterator + getUnwrappedIndex + getBufferIndexFromIndex
//
// Build:
//   cmake .. -DBUILD_BENCHMARKS=ON -DCMAKE_BUILD_TYPE=Release && make -j

#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "../lib/nanoPCL/benchmarks/common/benchmark_common.hpp"
#include "fastdem/elevation_map.hpp"
#include "fastdem/postprocess/feature_extraction.hpp"
#include "fastdem/postprocess/inpainting.hpp"

// Local config struct for benchmark inpainting variants (not part of public API)
namespace fastdem::config {
struct Inpainting {
  bool enabled = false;
  int max_iterations = 3;
  int min_valid_neighbors = 2;
};
}  // namespace fastdem::config
#include "fastdem/postprocess/spatial_smoothing.hpp"
#include "fastdem/postprocess/uncertainty_fusion.hpp"

#include <grid_map_core/GridMapMath.hpp>
#include <grid_map_core/iterators/CircleIterator.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <nanopcl/geometry/pca.hpp>

using namespace fastdem;

// ============================================================================
// Test Data Setup
// ============================================================================

ElevationMap createTestMap(int cells, float resolution, bool move_map = true) {
  float size = cells * resolution;
  ElevationMap map(size, size, resolution, "map");

  auto& elev = map.get(layer::elevation);
  std::mt19937 rng(42);
  std::normal_distribution<float> noise(0.0f, 0.02f);
  std::uniform_real_distribution<float> hole(0.0f, 1.0f);

  for (int r = 0; r < elev.rows(); ++r) {
    for (int c = 0; c < elev.cols(); ++c) {
      float x = c * resolution;
      float y = r * resolution;
      float z = 0.5f * std::sin(x * 2.0f) * std::cos(y * 1.5f) + noise(rng);
      elev(r, c) = (hole(rng) < 0.2f) ? NAN : z;
    }
  }

  // Non-zero startIndex (simulates robot-centric map that has moved)
  if (move_map) {
    grid_map::Position new_pos(size * 0.6, size * 0.4);
    map.move(new_pos);
  }
  return map;
}

void addKalmanLayers(ElevationMap& map) {
  const auto& elev = map.get(layer::elevation);
  map.add(layer::state, elev);
  map.add(layer::variance, NAN);
  auto& var_mat = map.get(layer::variance);

  std::mt19937 rng(123);
  std::uniform_real_distribution<float> vd(0.001f, 0.01f);
  for (int r = 0; r < elev.rows(); ++r)
    for (int c = 0; c < elev.cols(); ++c)
      if (std::isfinite(elev(r, c))) var_mat(r, c) = vd(rng);

  map.add(layer::upper_bound, NAN);
  map.add(layer::lower_bound, NAN);
  map.add(layer::uncertainty_range, NAN);
  map.add(layer::elevation_min, elev);
  map.add(layer::elevation_max, elev);
}

// ============================================================================
// Feature Extraction — Option A: CircleIterator
// ============================================================================

void featureExtraction_CircleIter(ElevationMap& map, float analysis_radius,
                                  int min_valid_neighbors) {
  if (!map.exists(layer::elevation)) return;
  if (!map.exists(layer::step)) map.add(layer::step, NAN);
  if (!map.exists(layer::slope)) map.add(layer::slope, NAN);
  if (!map.exists(layer::roughness)) map.add(layer::roughness, NAN);
  if (!map.exists(layer::curvature)) map.add(layer::curvature, NAN);
  if (!map.exists(layer::normal_x)) map.add(layer::normal_x, NAN);
  if (!map.exists(layer::normal_y)) map.add(layer::normal_y, NAN);
  if (!map.exists(layer::normal_z)) map.add(layer::normal_z, NAN);

  const auto& elev = map.get(layer::elevation);
  auto& step_mat = map.get(layer::step);
  auto& slope_mat = map.get(layer::slope);
  auto& roughness_mat = map.get(layer::roughness);
  auto& curvature_mat = map.get(layer::curvature);
  auto& nx_mat = map.get(layer::normal_x);
  auto& ny_mat = map.get(layer::normal_y);
  auto& nz_mat = map.get(layer::normal_z);

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const auto& idx = *it;
    const float center_z = elev(idx(0), idx(1));
    if (!std::isfinite(center_z)) continue;

    grid_map::Position center_pos;
    map.getPosition(idx, center_pos);

    Eigen::Vector3f sum = Eigen::Vector3f::Zero();
    Eigen::Matrix3f sum_sq = Eigen::Matrix3f::Zero();
    float z_min = std::numeric_limits<float>::max();
    float z_max = std::numeric_limits<float>::lowest();
    int count = 0;

    for (grid_map::CircleIterator cit(map, center_pos, analysis_radius);
         !cit.isPastEnd(); ++cit) {
      const auto& nidx = *cit;
      const float nz = elev(nidx(0), nidx(1));
      if (!std::isfinite(nz)) continue;

      grid_map::Position npos;
      map.getPosition(nidx, npos);

      const Eigen::Vector3f d(static_cast<float>(npos.x() - center_pos.x()),
                              static_cast<float>(npos.y() - center_pos.y()),
                              nz - center_z);
      sum += d;
      sum_sq.noalias() += d * d.transpose();
      z_min = std::min(z_min, nz);
      z_max = std::max(z_max, nz);
      ++count;
    }

    if (count < min_valid_neighbors) continue;

    const float inv_n = 1.0f / static_cast<float>(count);
    const Eigen::Vector3f mean = sum * inv_n;
    const Eigen::Matrix3f cov = sum_sq * inv_n - mean * mean.transpose();
    const auto pca = nanopcl::geometry::computePCA(cov);
    if (!pca.valid || pca.eigenvalues(1) < 1e-8f) continue;

    Eigen::Vector3f normal = pca.eigenvectors.col(0);
    if (normal.z() < 0.0f) normal = -normal;

    step_mat(idx(0), idx(1)) = z_max - z_min;
    slope_mat(idx(0), idx(1)) =
        std::acos(std::abs(normal.z())) * 180.0f / static_cast<float>(M_PI);
    roughness_mat(idx(0), idx(1)) = std::sqrt(pca.eigenvalues(0));
    curvature_mat(idx(0), idx(1)) = std::abs(pca.eigenvalues(0) / cov.trace());
    nx_mat(idx(0), idx(1)) = normal.x();
    ny_mat(idx(0), idx(1)) = normal.y();
    nz_mat(idx(0), idx(1)) = normal.z();
  }
}

// ============================================================================
// Feature Extraction — Option B: Precomputed + API
// ============================================================================

void featureExtraction_PrecomputedAPI(ElevationMap& map, float analysis_radius,
                                      int min_valid_neighbors) {
  if (!map.exists(layer::elevation)) return;
  if (!map.exists(layer::step)) map.add(layer::step, NAN);
  if (!map.exists(layer::slope)) map.add(layer::slope, NAN);
  if (!map.exists(layer::roughness)) map.add(layer::roughness, NAN);
  if (!map.exists(layer::curvature)) map.add(layer::curvature, NAN);
  if (!map.exists(layer::normal_x)) map.add(layer::normal_x, NAN);
  if (!map.exists(layer::normal_y)) map.add(layer::normal_y, NAN);
  if (!map.exists(layer::normal_z)) map.add(layer::normal_z, NAN);

  const auto& elev = map.get(layer::elevation);
  auto& step_mat = map.get(layer::step);
  auto& slope_mat = map.get(layer::slope);
  auto& roughness_mat = map.get(layer::roughness);
  auto& curvature_mat = map.get(layer::curvature);
  auto& nx_mat = map.get(layer::normal_x);
  auto& ny_mat = map.get(layer::normal_y);
  auto& nz_mat = map.get(layer::normal_z);
  const float resolution = map.getResolution();

  // Precompute offsets
  struct CircleOffset {
    int dr, dc;
    float dx, dy;
  };
  std::vector<CircleOffset> offsets;
  const int radius_cells =
      static_cast<int>(std::ceil(analysis_radius / resolution));
  const float radius_sq = analysis_radius * analysis_radius;
  for (int dr = -radius_cells; dr <= radius_cells; ++dr) {
    for (int dc = -radius_cells; dc <= radius_cells; ++dc) {
      float dist_sq =
          resolution * resolution * static_cast<float>(dr * dr + dc * dc);
      if (dist_sq <= radius_sq) {
        offsets.push_back({dr, dc, dc * resolution, -dr * resolution});
      }
    }
  }

  const auto& bufferSize = map.getSize();
  const auto& startIndex = map.getStartIndex();

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const auto& idx = *it;
    const float center_z = elev(idx(0), idx(1));
    if (!std::isfinite(center_z)) continue;

    const auto logical = it.getUnwrappedIndex();

    Eigen::Vector3f sum = Eigen::Vector3f::Zero();
    Eigen::Matrix3f sum_sq = Eigen::Matrix3f::Zero();
    float z_min = std::numeric_limits<float>::max();
    float z_max = std::numeric_limits<float>::lowest();
    int count = 0;

    for (const auto& [dr, dc, dx, dy] : offsets) {
      const int nlr = logical(0) + dr;
      const int nlc = logical(1) + dc;
      if (nlr < 0 || nlr >= bufferSize(0) || nlc < 0 || nlc >= bufferSize(1))
        continue;

      const auto nidx = grid_map::getBufferIndexFromIndex(
          grid_map::Index(nlr, nlc), bufferSize, startIndex);
      const float nz = elev(nidx(0), nidx(1));
      if (!std::isfinite(nz)) continue;

      const Eigen::Vector3f d(dx, dy, nz - center_z);
      sum += d;
      sum_sq.noalias() += d * d.transpose();
      z_min = std::min(z_min, nz);
      z_max = std::max(z_max, nz);
      ++count;
    }

    if (count < min_valid_neighbors) continue;

    const float inv_n = 1.0f / static_cast<float>(count);
    const Eigen::Vector3f mean = sum * inv_n;
    const Eigen::Matrix3f cov = sum_sq * inv_n - mean * mean.transpose();
    const auto pca = nanopcl::geometry::computePCA(cov);
    if (!pca.valid || pca.eigenvalues(1) < 1e-8f) continue;

    Eigen::Vector3f normal = pca.eigenvectors.col(0);
    if (normal.z() < 0.0f) normal = -normal;

    step_mat(idx(0), idx(1)) = z_max - z_min;
    slope_mat(idx(0), idx(1)) =
        std::acos(std::abs(normal.z())) * 180.0f / static_cast<float>(M_PI);
    roughness_mat(idx(0), idx(1)) = std::sqrt(pca.eigenvalues(0));
    curvature_mat(idx(0), idx(1)) = std::abs(pca.eigenvalues(0) / cov.trace());
    nx_mat(idx(0), idx(1)) = normal.x();
    ny_mat(idx(0), idx(1)) = normal.y();
    nz_mat(idx(0), idx(1)) = normal.z();
  }
}

// ============================================================================
// Feature Extraction — MapIndexer
// ============================================================================

void featureExtraction_MapIndexer(ElevationMap& map, float analysis_radius,
                                   int min_valid_neighbors) {
  if (!map.exists(layer::elevation)) return;
  if (!map.exists(layer::step)) map.add(layer::step, NAN);
  if (!map.exists(layer::slope)) map.add(layer::slope, NAN);
  if (!map.exists(layer::roughness)) map.add(layer::roughness, NAN);
  if (!map.exists(layer::curvature)) map.add(layer::curvature, NAN);
  if (!map.exists(layer::normal_x)) map.add(layer::normal_x, NAN);
  if (!map.exists(layer::normal_y)) map.add(layer::normal_y, NAN);
  if (!map.exists(layer::normal_z)) map.add(layer::normal_z, NAN);

  const auto& elev = map.get(layer::elevation);
  auto& step_mat = map.get(layer::step);
  auto& slope_mat = map.get(layer::slope);
  auto& roughness_mat = map.get(layer::roughness);
  auto& curvature_mat = map.get(layer::curvature);
  auto& nx_mat = map.get(layer::normal_x);
  auto& ny_mat = map.get(layer::normal_y);
  auto& nz_mat = map.get(layer::normal_z);

  const MapIndexer idx(map);
  const auto neighbors = idx.circleNeighbors(analysis_radius);

  for (int row = 0; row < idx.rows; ++row) {
    for (int col = 0; col < idx.cols; ++col) {
      auto [r, c] = idx(row, col);
      const float center_z = elev(r, c);
      if (!std::isfinite(center_z)) continue;

      Eigen::Vector3f sum = Eigen::Vector3f::Zero();
      Eigen::Matrix3f sum_sq = Eigen::Matrix3f::Zero();
      float z_min = std::numeric_limits<float>::max();
      float z_max = std::numeric_limits<float>::lowest();
      int count = 0;

      for (const auto& [dr, dc, dist_sq] : neighbors) {
        if (!idx.contains(row + dr, col + dc)) continue;
        auto [nr, nc] = idx(row + dr, col + dc);

        const float nz = elev(nr, nc);
        if (!std::isfinite(nz)) continue;

        const Eigen::Vector3f d(dc * idx.resolution, -dr * idx.resolution,
                                nz - center_z);
        sum += d;
        sum_sq.noalias() += d * d.transpose();
        z_min = std::min(z_min, nz);
        z_max = std::max(z_max, nz);
        ++count;
      }

      if (count < min_valid_neighbors) continue;

      const float inv_n = 1.0f / static_cast<float>(count);
      const Eigen::Vector3f mean = sum * inv_n;
      const Eigen::Matrix3f cov = sum_sq * inv_n - mean * mean.transpose();
      const auto pca = nanopcl::geometry::computePCA(cov);
      if (!pca.valid || pca.eigenvalues(1) < 1e-8f) continue;

      Eigen::Vector3f normal = pca.eigenvectors.col(0);
      if (normal.z() < 0.0f) normal = -normal;

      step_mat(r, c) = z_max - z_min;
      slope_mat(r, c) =
          std::acos(std::abs(normal.z())) * 180.0f / static_cast<float>(M_PI);
      roughness_mat(r, c) = std::sqrt(pca.eigenvalues(0));
      curvature_mat(r, c) = std::abs(pca.eigenvalues(0) / cov.trace());
      nx_mat(r, c) = normal.x();
      ny_mat(r, c) = normal.y();
      nz_mat(r, c) = normal.z();
    }
  }
}

// ============================================================================
// Inpainting — Option B: Position + getIndex
// ============================================================================

void inpainting_PositionAPI(ElevationMap& map,
                            const config::Inpainting& config) {
  if (!config.enabled) return;
  if (!map.exists(layer::elevation_inpainted))
    map.add(layer::elevation_inpainted, NAN);

  const auto& elevation = map.get(layer::elevation);
  auto& inpainted = map.get(layer::elevation_inpainted);
  inpainted = elevation;

  const double res = map.getResolution();
  constexpr double dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
  constexpr double dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};

  const int rows = inpainted.rows();
  const int cols = inpainted.cols();
  Eigen::MatrixXf buffer(rows, cols);

  for (int iter = 0; iter < config.max_iterations; ++iter) {
    bool changed = false;
    buffer = inpainted;

    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
      const auto& idx = *it;
      if (!std::isnan(inpainted(idx(0), idx(1)))) continue;

      grid_map::Position pos;
      map.getPosition(idx, pos);

      float sum = 0.0f;
      int count = 0;
      for (int i = 0; i < 8; ++i) {
        grid_map::Position npos(pos.x() + dx[i] * res, pos.y() + dy[i] * res);
        grid_map::Index nidx;
        if (!map.getIndex(npos, nidx)) continue;
        float val = inpainted(nidx(0), nidx(1));
        if (std::isfinite(val)) {
          sum += val;
          ++count;
        }
      }
      if (count >= config.min_valid_neighbors) {
        buffer(idx(0), idx(1)) = sum / static_cast<float>(count);
        changed = true;
      }
    }

    inpainted = buffer;
    if (!changed) break;
  }
}

// ============================================================================
// Inpainting — Option C: Precomputed + API
// ============================================================================

void inpainting_PrecomputedAPI(ElevationMap& map,
                               const config::Inpainting& config) {
  if (!config.enabled) return;
  if (!map.exists(layer::elevation_inpainted))
    map.add(layer::elevation_inpainted, NAN);

  const auto& elevation = map.get(layer::elevation);
  auto& inpainted = map.get(layer::elevation_inpainted);
  inpainted = elevation;

  constexpr int dr[] = {-1, -1, -1, 0, 0, 1, 1, 1};
  constexpr int dc[] = {-1, 0, 1, -1, 1, -1, 0, 1};

  const auto& bufferSize = map.getSize();
  const auto& startIndex = map.getStartIndex();
  const int rows = inpainted.rows();
  const int cols = inpainted.cols();
  Eigen::MatrixXf buffer(rows, cols);

  for (int iter = 0; iter < config.max_iterations; ++iter) {
    bool changed = false;
    buffer = inpainted;

    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
      const auto& idx = *it;
      if (!std::isnan(inpainted(idx(0), idx(1)))) continue;

      const auto logical = it.getUnwrappedIndex();

      float sum = 0.0f;
      int count = 0;
      for (int i = 0; i < 8; ++i) {
        const int nlr = logical(0) + dr[i];
        const int nlc = logical(1) + dc[i];
        if (nlr < 0 || nlr >= bufferSize(0) || nlc < 0 || nlc >= bufferSize(1))
          continue;
        const auto nidx = grid_map::getBufferIndexFromIndex(
            grid_map::Index(nlr, nlc), bufferSize, startIndex);
        float val = inpainted(nidx(0), nidx(1));
        if (std::isfinite(val)) {
          sum += val;
          ++count;
        }
      }
      if (count >= config.min_valid_neighbors) {
        buffer(idx(0), idx(1)) = sum / static_cast<float>(count);
        changed = true;
      }
    }

    inpainted = buffer;
    if (!changed) break;
  }
}

// ============================================================================
// Spatial Smoothing — Option C: Precomputed + API
// ============================================================================

void spatialSmoothing_PrecomputedAPI(ElevationMap& map,
                                     const std::string& layer_name,
                                     int kernel_size = 3,
                                     int min_valid_neighbors = 5) {
  if (!map.exists(layer_name)) return;

  const Eigen::MatrixXf input = map.get(layer_name);
  auto& output = map.get(layer_name);

  const int half = kernel_size / 2;
  const auto& bufferSize = map.getSize();
  const auto& startIndex = map.getStartIndex();

  std::vector<float> window;
  window.reserve(kernel_size * kernel_size);

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const auto& idx = *it;
    if (!std::isfinite(input(idx(0), idx(1)))) continue;

    const auto logical = it.getUnwrappedIndex();

    window.clear();
    for (int dr = -half; dr <= half; ++dr) {
      for (int dc = -half; dc <= half; ++dc) {
        const int nlr = logical(0) + dr;
        const int nlc = logical(1) + dc;
        if (nlr < 0 || nlr >= bufferSize(0) || nlc < 0 || nlc >= bufferSize(1))
          continue;
        const auto nidx = grid_map::getBufferIndexFromIndex(
            grid_map::Index(nlr, nlc), bufferSize, startIndex);
        float val = input(nidx(0), nidx(1));
        if (std::isfinite(val)) window.push_back(val);
      }
    }

    if (static_cast<int>(window.size()) < min_valid_neighbors) continue;

    size_t mid = window.size() / 2;
    std::nth_element(window.begin(), window.begin() + mid, window.end());
    output(idx(0), idx(1)) = window[mid];
  }
}

// ============================================================================
// Inpainting — MapIndexer
// ============================================================================

void inpainting_MapIndexer(ElevationMap& map,
                            const config::Inpainting& config) {
  if (!config.enabled) return;
  if (!map.exists(layer::elevation_inpainted))
    map.add(layer::elevation_inpainted, NAN);

  const auto& elevation = map.get(layer::elevation);
  auto& inpainted = map.get(layer::elevation_inpainted);
  inpainted = elevation;

  constexpr int dr[] = {-1, -1, -1, 0, 0, 1, 1, 1};
  constexpr int dc[] = {-1, 0, 1, -1, 1, -1, 0, 1};

  const MapIndexer idx(map);
  Eigen::MatrixXf buffer(idx.rows, idx.cols);

  for (int iter = 0; iter < config.max_iterations; ++iter) {
    bool changed = false;
    buffer = inpainted;

    for (int row = 0; row < idx.rows; ++row) {
      for (int col = 0; col < idx.cols; ++col) {
        auto [r, c] = idx(row, col);
        if (!std::isnan(inpainted(r, c))) continue;

        float sum = 0.0f;
        int count = 0;
        for (int i = 0; i < 8; ++i) {
          if (!idx.contains(row + dr[i], col + dc[i])) continue;
          auto [nr, nc] = idx(row + dr[i], col + dc[i]);
          float val = inpainted(nr, nc);
          if (std::isfinite(val)) {
            sum += val;
            ++count;
          }
        }
        if (count >= config.min_valid_neighbors) {
          buffer(r, c) = sum / static_cast<float>(count);
          changed = true;
        }
      }
    }

    inpainted = buffer;
    if (!changed) break;
  }
}

// ============================================================================
// Spatial Smoothing — MapIndexer
// ============================================================================

void spatialSmoothing_MapIndexer(ElevationMap& map,
                                  const std::string& layer_name,
                                  int kernel_size = 3,
                                  int min_valid_neighbors = 5) {
  if (!map.exists(layer_name)) return;

  const Eigen::MatrixXf input = map.get(layer_name);
  auto& output = map.get(layer_name);
  const int half = kernel_size / 2;

  const MapIndexer idx(map);
  std::vector<float> window;
  window.reserve(kernel_size * kernel_size);

  for (int row = 0; row < idx.rows; ++row) {
    for (int col = 0; col < idx.cols; ++col) {
      auto [r, c] = idx(row, col);
      if (!std::isfinite(input(r, c))) continue;

      window.clear();
      for (int dr = -half; dr <= half; ++dr) {
        for (int dc = -half; dc <= half; ++dc) {
          if (!idx.contains(row + dr, col + dc)) continue;
          auto [nr, nc] = idx(row + dr, col + dc);
          float val = input(nr, nc);
          if (std::isfinite(val)) window.push_back(val);
        }
      }

      if (static_cast<int>(window.size()) < min_valid_neighbors) continue;

      size_t mid = window.size() / 2;
      std::nth_element(window.begin(), window.begin() + mid, window.end());
      output(r, c) = window[mid];
    }
  }
}

// ============================================================================
// Helpers
// ============================================================================

void resetFeatureLayers(ElevationMap& map) {
  for (const auto* name :
       {layer::step, layer::slope, layer::roughness, layer::curvature,
        layer::normal_x, layer::normal_y, layer::normal_z}) {
    if (map.exists(name)) map.get(name).setConstant(NAN);
  }
}

// ============================================================================
// Main
// ============================================================================

int main() {
  benchmark::printHeader("Postprocess Iteration Pattern Benchmark");
  benchmark::PlatformInfo::capture().print();

  constexpr int N = 150;
  constexpr float RES = 0.05f;

  std::cout << "\nMap: " << N << "x" << N << " (" << RES
            << "m/cell, ~20% NaN, startIndex != 0)\n";

  // ==========================================================================
  // 1. Iteration-only microbenchmark (pure overhead)
  // ==========================================================================
  benchmark::printSection("1. Iteration-only (read all cells, no computation)");
  {
    auto map = createTestMap(N, RES);
    const auto& elev = map.get(layer::elevation);
    const int rows = elev.rows();
    const int cols = elev.cols();
    const auto& si = map.getStartIndex();
    const int sr = si(0), sc = si(1);

    std::cout << "  startIndex = (" << sr << ", " << sc << ")\n\n";

    volatile float sink = 0;

    // Manual lr/lc
    auto s0 = benchmark::runVoid(
        [&]() {
          float acc = 0;
          for (int lr = 0; lr < rows; ++lr) {
            for (int lc = 0; lc < cols; ++lc) {
              int r = lr + sr;
              if (r >= rows) r -= rows;
              int c = lc + sc;
              if (c >= cols) c -= cols;
              acc += elev(r, c);
            }
          }
          sink = acc;
        },
        100);
    benchmark::printResult("Manual lr/lc", s0);

    // GridMapIterator (storage-order)
    auto s1 = benchmark::runVoid(
        [&]() {
          float acc = 0;
          for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
            const auto idx = *it;
            acc += elev(idx(0), idx(1));
          }
          sink = acc;
        },
        100);
    benchmark::printResultWithSpeedup("GridMapIterator", s1, s0.mean);

    // GridMapIterator + 1 neighbor via API
    const auto& bufSz = map.getSize();
    const auto& startIdx = map.getStartIndex();

    auto s2 = benchmark::runVoid(
        [&]() {
          float acc = 0;
          for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
            const auto idx = *it;
            acc += elev(idx(0), idx(1));
            const auto lg = it.getUnwrappedIndex();
            int nr = lg(0) + 1, nc = lg(1) + 1;
            if (nr >= 0 && nr < bufSz(0) && nc >= 0 && nc < bufSz(1)) {
              auto ni = grid_map::getBufferIndexFromIndex(
                  grid_map::Index(nr, nc), bufSz, startIdx);
              acc += elev(ni(0), ni(1));
            }
          }
          sink = acc;
        },
        100);
    benchmark::printResultWithSpeedup("GMI + 1 neighbor (API)", s2, s0.mean);

    // Manual lr/lc + 1 neighbor (manual wrapping)
    auto s3 = benchmark::runVoid(
        [&]() {
          float acc = 0;
          for (int lr = 0; lr < rows; ++lr) {
            for (int lc = 0; lc < cols; ++lc) {
              int r = lr + sr;
              if (r >= rows) r -= rows;
              int c = lc + sc;
              if (c >= cols) c -= cols;
              acc += elev(r, c);
              int nlr = lr + 1, nlc = lc + 1;
              if (nlr >= 0 && nlr < rows && nlc >= 0 && nlc < cols) {
                int nr = nlr + sr;
                if (nr >= rows) nr -= rows;
                int nc = nlc + sc;
                if (nc >= cols) nc -= cols;
                acc += elev(nr, nc);
              }
            }
          }
          sink = acc;
        },
        100);
    benchmark::printResultWithSpeedup("Manual + 1 neighbor", s3, s0.mean);

    // MapIndexer (outer loop only)
    MapIndexer idx(map);
    auto s4 = benchmark::runVoid(
        [&]() {
          float acc = 0;
          for (int row = 0; row < idx.rows; ++row) {
            for (int col = 0; col < idx.cols; ++col) {
              auto [r, c] = idx(row, col);
              acc += elev(r, c);
            }
          }
          sink = acc;
        },
        100);
    benchmark::printResultWithSpeedup("MapIndexer", s4, s0.mean);

    // MapIndexer + 1 neighbor
    auto s5 = benchmark::runVoid(
        [&]() {
          float acc = 0;
          for (int row = 0; row < idx.rows; ++row) {
            for (int col = 0; col < idx.cols; ++col) {
              auto [r, c] = idx(row, col);
              acc += elev(r, c);
              if (idx.contains(row + 1, col + 1)) {
                auto [nr, nc] = idx(row + 1, col + 1);
                acc += elev(nr, nc);
              }
            }
          }
          sink = acc;
        },
        100);
    benchmark::printResultWithSpeedup("MapIndexer + 1 neighbor", s5, s0.mean);
  }

  // ==========================================================================
  // 2. Feature Extraction (heaviest: radius=0.3m → ~113 neighbors + PCA)
  // ==========================================================================
  benchmark::printSection("2. Feature Extraction (radius=0.3m, full PCA)");
  {
    constexpr float RADIUS = 0.3f;
    constexpr int MIN_N = 4;

    auto map0 = createTestMap(N, RES);
    auto map1 = createTestMap(N, RES);
    auto map2 = createTestMap(N, RES);
    auto map3 = createTestMap(N, RES);

    auto s0 = benchmark::runVoid(
        [&]() {
          resetFeatureLayers(map0);
          applyFeatureExtraction(map0, RADIUS, MIN_N);
        },
        30);
    benchmark::printResult("Current (manual lr/lc)", s0);

    auto s1 = benchmark::runVoid(
        [&]() {
          resetFeatureLayers(map1);
          featureExtraction_CircleIter(map1, RADIUS, MIN_N);
        },
        30);
    benchmark::printResultWithSpeedup("A: CircleIterator", s1, s0.mean);

    auto s2 = benchmark::runVoid(
        [&]() {
          resetFeatureLayers(map2);
          featureExtraction_PrecomputedAPI(map2, RADIUS, MIN_N);
        },
        30);
    benchmark::printResultWithSpeedup("B: Precomputed + API", s2, s0.mean);

    auto s3 = benchmark::runVoid(
        [&]() {
          resetFeatureLayers(map3);
          featureExtraction_MapIndexer(map3, RADIUS, MIN_N);
        },
        30);
    benchmark::printResultWithSpeedup("MapIndexer", s3, s0.mean);
  }

  // ==========================================================================
  // 3. Inpainting (8-neighbor, 3 iterations)
  // ==========================================================================
  benchmark::printSection("3. Inpainting (8-neighbor, 3 iterations)");
  {
    config::Inpainting cfg;
    cfg.enabled = true;
    cfg.max_iterations = 3;
    cfg.min_valid_neighbors = 2;

    auto map0 = createTestMap(N, RES);
    auto map1 = createTestMap(N, RES);
    auto map2 = createTestMap(N, RES);

    auto s0 = benchmark::runVoid(
        [&]() {
          if (map0.exists(layer::elevation_inpainted))
            map0.get(layer::elevation_inpainted).setConstant(NAN);
          applyInpainting(map0, cfg.max_iterations, cfg.min_valid_neighbors);
        },
        50);
    benchmark::printResult("Current (manual lr/lc)", s0);

    auto s1 = benchmark::runVoid(
        [&]() {
          if (map1.exists(layer::elevation_inpainted))
            map1.get(layer::elevation_inpainted).setConstant(NAN);
          inpainting_PositionAPI(map1, cfg);
        },
        50);
    benchmark::printResultWithSpeedup("B: Position + getIndex", s1, s0.mean);

    auto s2 = benchmark::runVoid(
        [&]() {
          if (map2.exists(layer::elevation_inpainted))
            map2.get(layer::elevation_inpainted).setConstant(NAN);
          inpainting_PrecomputedAPI(map2, cfg);
        },
        50);
    benchmark::printResultWithSpeedup("C: Precomputed + API", s2, s0.mean);

    auto map3 = createTestMap(N, RES);
    auto s3 = benchmark::runVoid(
        [&]() {
          if (map3.exists(layer::elevation_inpainted))
            map3.get(layer::elevation_inpainted).setConstant(NAN);
          inpainting_MapIndexer(map3, cfg);
        },
        50);
    benchmark::printResultWithSpeedup("MapIndexer", s3, s0.mean);
  }

  // ==========================================================================
  // 4. Spatial Smoothing (3x3 median)
  // ==========================================================================
  benchmark::printSection("4. Spatial Smoothing (3x3 median)");
  {
    auto map0 = createTestMap(N, RES);
    auto map1 = createTestMap(N, RES);
    const Eigen::MatrixXf original = map0.get(layer::elevation);

    auto s0 = benchmark::runVoid(
        [&]() {
          map0.get(layer::elevation) = original;
          applySpatialSmoothing(map0, layer::elevation, 3, 5);
        },
        50);
    benchmark::printResult("Current (manual lr/lc)", s0);

    auto s1 = benchmark::runVoid(
        [&]() {
          map1.get(layer::elevation) = original;
          spatialSmoothing_PrecomputedAPI(map1, layer::elevation, 3, 5);
        },
        50);
    benchmark::printResultWithSpeedup("C: Precomputed + API", s1, s0.mean);

    auto map2 = createTestMap(N, RES);
    auto s2 = benchmark::runVoid(
        [&]() {
          map2.get(layer::elevation) = original;
          spatialSmoothing_MapIndexer(map2, layer::elevation, 3, 5);
        },
        50);
    benchmark::printResultWithSpeedup("MapIndexer", s2, s0.mean);
  }

  // ==========================================================================
  // 5. Uncertainty Fusion (for reference — shows absolute cost)
  // ==========================================================================
  benchmark::printSection("5. Uncertainty Fusion (radius=0.15m, reference)");
  {
    config::UncertaintyFusion cfg;
    cfg.enabled = true;
    cfg.search_radius = 0.15f;
    cfg.spatial_sigma = 0.05f;
    cfg.min_valid_neighbors = 3;

    auto map = createTestMap(N, RES);
    addKalmanLayers(map);

    auto s0 = benchmark::runVoid(
        [&]() {
          map.get(layer::upper_bound).setConstant(NAN);
          map.get(layer::lower_bound).setConstant(NAN);
          map.get(layer::uncertainty_range).setConstant(NAN);
          applyUncertaintyFusion(map, cfg);
        },
        30);
    benchmark::printResult("Current (manual lr/lc)", s0);
    std::cout
        << "  (iteration overhead same as feature_extraction benchmark)\n";
  }

  benchmark::printFooter();
  return 0;
}
