// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT
//
// Trajectory file I/O and manipulation for SLAM applications.
//
// Supported formats:
//   - TUM: timestamp tx ty tz qx qy qz qw
//   - KITTI: 3x4 transformation matrix (row-major)
//
// Features:
//   - Pose interpolation (Slerp for rotation)
//   - Trajectory transformation
//   - Statistics (length, duration)
//
// Example:
//   Trajectory traj = io::loadTrajectoryTUM("trajectory.txt");
//   auto pose = traj.poseAt(1000.5);  // Interpolated pose
//   traj.transform(T_world_to_map);

#ifndef NANOPCL_IO_TRAJECTORY_IO_HPP
#define NANOPCL_IO_TRAJECTORY_IO_HPP

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "nanopcl/io/pcd_io.hpp" // For IOException

namespace nanopcl {
namespace io {

/// @brief Timestamped pose for trajectory representation
struct StampedPose {
  double timestamp;
  Eigen::Isometry3d pose;

  StampedPose()
      : timestamp(0.0), pose(Eigen::Isometry3d::Identity()) {}
  StampedPose(double t, const Eigen::Isometry3d& p)
      : timestamp(t), pose(p) {}
};

// =================
// Trajectory Class
// =================

/// @brief Trajectory with interpolation and transformation support
class Trajectory {
public:
  using iterator = std::vector<StampedPose>::iterator;
  using const_iterator = std::vector<StampedPose>::const_iterator;

  Trajectory() = default;
  explicit Trajectory(std::vector<StampedPose> poses)
      : poses_(std::move(poses)) {}

  [[nodiscard]] size_t size() const { return poses_.size(); }
  [[nodiscard]] bool empty() const { return poses_.empty(); }

  void push_back(const StampedPose& sp) { poses_.push_back(sp); }
  void push_back(double t, const Eigen::Isometry3d& p) {
    poses_.emplace_back(t, p);
  }

  void reserve(size_t n) { poses_.reserve(n); }
  void clear() { poses_.clear(); }

  StampedPose& operator[](size_t i) { return poses_[i]; }
  const StampedPose& operator[](size_t i) const { return poses_[i]; }

  StampedPose& front() { return poses_.front(); }
  const StampedPose& front() const { return poses_.front(); }
  StampedPose& back() { return poses_.back(); }
  const StampedPose& back() const { return poses_.back(); }

  iterator begin() { return poses_.begin(); }
  iterator end() { return poses_.end(); }
  const_iterator begin() const { return poses_.begin(); }
  const_iterator end() const { return poses_.end(); }

  // ==============
  // Interpolation
  // ==============

  /// @brief Get interpolated pose at given timestamp
  /// @param t Query timestamp
  /// @return Interpolated pose (Slerp for rotation, linear for translation)
  /// @throws IOException if trajectory is empty or timestamp out of range
  [[nodiscard]] Eigen::Isometry3d poseAt(double t) const {
    if (poses_.empty()) {
      throw IOException("Cannot interpolate: trajectory is empty");
    }

    // Find bracketing poses
    auto it = std::lower_bound(
        poses_.begin(), poses_.end(), t, [](const StampedPose& sp, double ts) { return sp.timestamp < ts; });

    // Exact match or before first
    if (it == poses_.begin()) {
      return poses_.front().pose;
    }
    // After last
    if (it == poses_.end()) {
      return poses_.back().pose;
    }

    // Interpolate between (it-1) and it
    const auto& p0 = *(it - 1);
    const auto& p1 = *it;

    double alpha = (t - p0.timestamp) / (p1.timestamp - p0.timestamp);
    alpha = std::clamp(alpha, 0.0, 1.0);

    return interpolate(p0.pose, p1.pose, alpha);
  }

  // ===============
  // Transformation
  // ===============

  /// @brief Transform all poses by a rigid transformation
  /// @param T Transformation to apply (new_pose = T * old_pose)
  void transform(const Eigen::Isometry3d& T) {
    for (auto& sp : poses_) {
      sp.pose = T * sp.pose;
    }
  }

  /// @brief Transform all poses by a rigid transformation (right multiply)
  /// @param T Transformation to apply (new_pose = old_pose * T)
  void transformRight(const Eigen::Isometry3d& T) {
    for (auto& sp : poses_) {
      sp.pose = sp.pose * T;
    }
  }

  // ===========
  // Statistics
  // ===========

  /// @brief Compute total trajectory length (sum of translation distances)
  /// @return Total distance traveled in meters
  [[nodiscard]] double length() const {
    if (poses_.size() < 2) return 0.0;

    double total = 0.0;
    for (size_t i = 1; i < poses_.size(); ++i) {
      total += (poses_[i].pose.translation() - poses_[i - 1].pose.translation())
                   .norm();
    }
    return total;
  }

  /// @brief Get total duration of trajectory
  /// @return Duration in seconds
  [[nodiscard]] double duration() const {
    if (poses_.size() < 2) return 0.0;
    return poses_.back().timestamp - poses_.front().timestamp;
  }

  /// @brief Get start timestamp
  [[nodiscard]] double startTime() const {
    return poses_.empty() ? 0.0 : poses_.front().timestamp;
  }

  /// @brief Get end timestamp
  [[nodiscard]] double endTime() const {
    return poses_.empty() ? 0.0 : poses_.back().timestamp;
  }

  // =======
  // Access
  // =======

  std::vector<StampedPose>& poses() { return poses_; }
  const std::vector<StampedPose>& poses() const { return poses_; }

private:
  std::vector<StampedPose> poses_;

  /// @brief Interpolate between two poses using Slerp
  static Eigen::Isometry3d interpolate(const Eigen::Isometry3d& p0,
                                       const Eigen::Isometry3d& p1,
                                       double alpha) {
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();

    // Linear interpolation for translation
    result.translation() =
        (1.0 - alpha) * p0.translation() + alpha * p1.translation();

    // Slerp for rotation
    Eigen::Quaterniond q0(p0.rotation());
    Eigen::Quaterniond q1(p1.rotation());
    result.linear() = q0.slerp(alpha, q1).toRotationMatrix();

    return result;
  }
};

// ============================================
// TUM Format (timestamp tx ty tz qx qy qz qw)
// ============================================

/// @brief Load trajectory from TUM format stream
inline Trajectory loadTrajectoryTUM(std::istream& is) {
  if (!is) {
    throw IOException("Invalid input stream");
  }

  Trajectory trajectory;
  std::string line;

  while (std::getline(is, line)) {
    if (line.empty() || line[0] == '#') continue;

    std::istringstream iss(line);
    double timestamp, tx, ty, tz, qx, qy, qz, qw;

    if (!(iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {
      continue;
    }

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(tx, ty, tz);
    pose.linear() = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();

    trajectory.push_back(timestamp, pose);
  }

  return trajectory;
}

/// @brief Load trajectory from TUM format file
inline Trajectory loadTrajectoryTUM(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs) {
    throw IOException("Cannot open file: " + path);
  }
  return loadTrajectoryTUM(ifs);
}

/// @brief Save trajectory to TUM format stream
inline void saveTrajectoryTUM(std::ostream& os,
                              const Trajectory& trajectory,
                              int precision = 9) {
  if (!os) {
    throw IOException("Invalid output stream");
  }

  os << std::fixed << std::setprecision(precision);

  for (const auto& sp : trajectory) {
    Eigen::Quaterniond q(sp.pose.rotation());
    const auto& t = sp.pose.translation();

    os << sp.timestamp << " " << t.x() << " " << t.y() << " " << t.z() << " "
       << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
  }

  if (!os) {
    throw IOException("Error writing trajectory");
  }
}

/// @brief Save trajectory to TUM format file
inline void saveTrajectoryTUM(const std::string& path,
                              const Trajectory& trajectory,
                              int precision = 9) {
  std::ofstream ofs(path);
  if (!ofs) {
    throw IOException("Cannot create file: " + path);
  }
  saveTrajectoryTUM(ofs, trajectory, precision);
}

// ====================================================
// KITTI Format (3x4 transformation matrix, row-major)
// ====================================================

/// @brief Load trajectory from KITTI format stream
inline Trajectory loadTrajectoryKITTI(std::istream& is) {
  if (!is) {
    throw IOException("Invalid input stream");
  }

  Trajectory trajectory;
  std::string line;
  double timestamp = 0.0;

  while (std::getline(is, line)) {
    if (line.empty()) continue;

    std::istringstream iss(line);
    double m[12];

    bool valid = true;
    for (int i = 0; i < 12 && valid; ++i) {
      if (!(iss >> m[i])) valid = false;
    }

    if (!valid) continue;

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 0) = m[0];
    T(0, 1) = m[1];
    T(0, 2) = m[2];
    T(0, 3) = m[3];
    T(1, 0) = m[4];
    T(1, 1) = m[5];
    T(1, 2) = m[6];
    T(1, 3) = m[7];
    T(2, 0) = m[8];
    T(2, 1) = m[9];
    T(2, 2) = m[10];
    T(2, 3) = m[11];

    Eigen::Isometry3d pose;
    pose.matrix() = T;

    trajectory.push_back(timestamp, pose);
    timestamp += 1.0;
  }

  return trajectory;
}

/// @brief Load trajectory from KITTI format file
inline Trajectory loadTrajectoryKITTI(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs) {
    throw IOException("Cannot open file: " + path);
  }
  return loadTrajectoryKITTI(ifs);
}

/// @brief Save trajectory to KITTI format stream
inline void saveTrajectoryKITTI(std::ostream& os,
                                const Trajectory& trajectory,
                                int precision = 9) {
  if (!os) {
    throw IOException("Invalid output stream");
  }

  os << std::fixed << std::setprecision(precision);

  for (const auto& sp : trajectory) {
    const Eigen::Matrix4d& T = sp.pose.matrix();

    os << T(0, 0) << " " << T(0, 1) << " " << T(0, 2) << " " << T(0, 3) << " "
       << T(1, 0) << " " << T(1, 1) << " " << T(1, 2) << " " << T(1, 3) << " "
       << T(2, 0) << " " << T(2, 1) << " " << T(2, 2) << " " << T(2, 3) << "\n";
  }

  if (!os) {
    throw IOException("Error writing trajectory");
  }
}

/// @brief Save trajectory to KITTI format file
inline void saveTrajectoryKITTI(const std::string& path,
                                const Trajectory& trajectory,
                                int precision = 9) {
  std::ofstream ofs(path);
  if (!ofs) {
    throw IOException("Cannot create file: " + path);
  }
  saveTrajectoryKITTI(ofs, trajectory, precision);
}

// ===========================================
// Convenience: Load poses without timestamps
// ===========================================

/// @brief Load poses only (without timestamps)
inline std::vector<Eigen::Isometry3d> loadPoses(const std::string& path,
                                                const std::string& format = "tum") {
  Trajectory traj;
  if (format == "kitti") {
    traj = loadTrajectoryKITTI(path);
  } else {
    traj = loadTrajectoryTUM(path);
  }

  std::vector<Eigen::Isometry3d> poses;
  poses.reserve(traj.size());
  for (const auto& sp : traj) {
    poses.push_back(sp.pose);
  }
  return poses;
}

} // namespace io
} // namespace nanopcl

#endif // NANOPCL_IO_TRAJECTORY_IO_HPP
