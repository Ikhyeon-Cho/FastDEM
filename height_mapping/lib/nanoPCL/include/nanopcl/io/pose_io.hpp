// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_IO_POSE_IO_HPP
#define NANOPCL_IO_POSE_IO_HPP

#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "nanopcl/io/error.hpp"
#include "nanopcl/transform/se3.hpp"

namespace nanopcl {
namespace io {

/**
 * @brief Save a single SE3 pose to TUM trajectory format.
 *
 * TUM format: `timestamp tx ty tz qx qy qz qw`
 * Compatible with evo (https://github.com/MichaelGrupp/evo) evaluation tool.
 *
 * @tparam Scalar Floating point type (double or float)
 * @param filename Output file path
 * @param pose SE3 pose to save
 * @param timestamp_sec Timestamp in seconds
 * @param append Append to file if true, overwrite if false (default: false)
 * @return true if successful
 */
template <typename Scalar = double>
inline bool savePoseTUM(const std::string& filename, const SE3_<Scalar>& pose,
                        double timestamp_sec, bool append = false) {
  auto mode = append ? (std::ios::out | std::ios::app) : std::ios::out;
  std::ofstream fs(filename, mode);
  if (!fs.is_open()) {
    detail::setLastError("Cannot open file: " + filename);
    return false;
  }

  // Write header comment only for new files
  if (!append) {
    fs << "# timestamp tx ty tz qx qy qz qw\n";
  }

  auto t = pose.translation();
  auto q = pose.quaternion();

  // Full precision for timestamps and poses
  fs << std::fixed << std::setprecision(9) << timestamp_sec << " "
     << std::setprecision(6) << static_cast<double>(t.x()) << " "
     << static_cast<double>(t.y()) << " " << static_cast<double>(t.z()) << " "
     << static_cast<double>(q.x()) << " " << static_cast<double>(q.y()) << " "
     << static_cast<double>(q.z()) << " " << static_cast<double>(q.w()) << "\n";

  return true;
}

/**
 * @brief Save multiple SE3 poses as TUM trajectory.
 *
 * @tparam Scalar Floating point type (double or float)
 * @param filename Output file path
 * @param trajectory Vector of (timestamp_sec, pose) pairs
 * @return true if successful
 */
template <typename Scalar = double>
inline bool saveTrajectoryTUM(
    const std::string& filename,
    const std::vector<std::pair<double, SE3_<Scalar>>>& trajectory) {
  std::ofstream fs(filename);
  if (!fs.is_open()) {
    detail::setLastError("Cannot open file: " + filename);
    return false;
  }

  fs << "# timestamp tx ty tz qx qy qz qw\n";

  for (const auto& [timestamp_sec, pose] : trajectory) {
    auto t = pose.translation();
    auto q = pose.quaternion();

    fs << std::fixed << std::setprecision(9) << timestamp_sec << " "
       << std::setprecision(6) << static_cast<double>(t.x()) << " "
       << static_cast<double>(t.y()) << " " << static_cast<double>(t.z()) << " "
       << static_cast<double>(q.x()) << " " << static_cast<double>(q.y()) << " "
       << static_cast<double>(q.z()) << " " << static_cast<double>(q.w())
       << "\n";
  }

  return true;
}

/**
 * @brief Load a single SE3 pose from TUM trajectory format.
 *
 * Reads the first non-comment line from the file.
 *
 * @tparam Scalar Floating point type (double or float)
 * @param filename Input file path
 * @param pose [out] Loaded SE3 pose
 * @param timestamp_sec [out] Loaded timestamp in seconds
 * @return true if successful
 */
template <typename Scalar = double>
inline bool loadPoseTUM(const std::string& filename, SE3_<Scalar>& pose,
                        double& timestamp_sec) {
  std::ifstream fs(filename);
  if (!fs.is_open()) {
    detail::setLastError("Cannot open file: " + filename);
    return false;
  }

  std::string line;
  while (std::getline(fs, line)) {
    // Skip empty lines and comments
    if (line.empty() || line[0] == '#') continue;

    std::stringstream ss(line);
    double ts, tx, ty, tz, qx, qy, qz, qw;

    if (!(ss >> ts >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {
      detail::setLastError("Invalid TUM format in: " + filename);
      return false;
    }

    timestamp_sec = ts;
    pose = SE3_<Scalar>::fromQuaternion(
        static_cast<Scalar>(qx), static_cast<Scalar>(qy),
        static_cast<Scalar>(qz), static_cast<Scalar>(qw),
        typename SE3_<Scalar>::Vector3(static_cast<Scalar>(tx),
                                       static_cast<Scalar>(ty),
                                       static_cast<Scalar>(tz)));
    return true;
  }

  detail::setLastError("No valid pose data in: " + filename);
  return false;
}

/**
 * @brief Load multiple SE3 poses from TUM trajectory format.
 *
 * @tparam Scalar Floating point type (double or float)
 * @param filename Input file path
 * @param trajectory [out] Loaded vector of (timestamp_sec, pose) pairs
 * @return true if successful
 */
template <typename Scalar = double>
inline bool loadTrajectoryTUM(
    const std::string& filename,
    std::vector<std::pair<double, SE3_<Scalar>>>& trajectory) {
  std::ifstream fs(filename);
  if (!fs.is_open()) {
    detail::setLastError("Cannot open file: " + filename);
    return false;
  }

  trajectory.clear();

  std::string line;
  while (std::getline(fs, line)) {
    // Skip empty lines and comments
    if (line.empty() || line[0] == '#') continue;

    std::stringstream ss(line);
    double ts, tx, ty, tz, qx, qy, qz, qw;

    if (!(ss >> ts >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {
      // Skip invalid lines silently (common in trajectory files)
      continue;
    }

    auto pose = SE3_<Scalar>::fromQuaternion(
        static_cast<Scalar>(qx), static_cast<Scalar>(qy),
        static_cast<Scalar>(qz), static_cast<Scalar>(qw),
        typename SE3_<Scalar>::Vector3(static_cast<Scalar>(tx),
                                       static_cast<Scalar>(ty),
                                       static_cast<Scalar>(tz)));
    trajectory.emplace_back(ts, pose);
  }

  if (trajectory.empty()) {
    detail::setLastError("No valid poses loaded from: " + filename);
    return false;
  }

  return true;
}

}  // namespace io
}  // namespace nanopcl

#endif  // NANOPCL_IO_POSE_IO_HPP
