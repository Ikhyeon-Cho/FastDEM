// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Trajectory I/O for Transform sequences.
//
// Supports TUM trajectory format for evaluation with evo tool.
// Format: `timestamp tx ty tz qx qy qz qw`

#ifndef NANOPCL_IO_TRAJECTORY_IO_HPP
#define NANOPCL_IO_TRAJECTORY_IO_HPP

#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "nanopcl/io/error.hpp"
#include "nanopcl/transform/transform.hpp"

namespace npcl {
namespace io {

// =============================================================================
// Save Transform
// =============================================================================

/**
 * @brief Save a single Transform to TUM trajectory format
 *
 * TUM format: `timestamp tx ty tz qx qy qz qw`
 * Compatible with evo (https://github.com/MichaelGrupp/evo) evaluation tool.
 *
 * @param filename Output file path
 * @param tf Transform to save (uses internal timestamp)
 * @param append Append to file if true, overwrite if false (default: false)
 * @throws IOError if file cannot be opened
 *
 * @code
 *   Transform tf("map", "base_link");
 *   tf.setTimestamp(time::now());
 *   io::saveTransformTUM("pose.txt", tf);
 * @endcode
 */
template <typename Scalar = double>
inline void saveTransformTUM(const std::string& filename,
                             const Transform_<Scalar>& tf,
                             bool append = false) {
  auto mode = append ? (std::ios::out | std::ios::app) : std::ios::out;
  std::ofstream fs(filename, mode);
  if (!fs.is_open()) {
    throw IOError("Cannot open file for writing: " + filename);
  }

  if (!append) {
    fs << "# timestamp tx ty tz qx qy qz qw\n";
  }

  double timestamp_sec = time::toSec(tf.timestamp());
  auto t = tf.translation();
  auto q = tf.quaternion();

  fs << std::fixed << std::setprecision(9) << timestamp_sec << " "
     << std::setprecision(6) << static_cast<double>(t.x()) << " "
     << static_cast<double>(t.y()) << " " << static_cast<double>(t.z()) << " "
     << static_cast<double>(q.x()) << " " << static_cast<double>(q.y()) << " "
     << static_cast<double>(q.z()) << " " << static_cast<double>(q.w()) << "\n";
}

/**
 * @brief Save multiple Transforms as TUM trajectory
 *
 * @param filename Output file path
 * @param trajectory Vector of Transforms (each with its own timestamp)
 * @throws IOError if file cannot be opened
 *
 * @code
 *   std::vector<Transform> trajectory;
 *   // ... populate trajectory ...
 *   io::saveTrajectoryTUM("trajectory.txt", trajectory);
 * @endcode
 */
template <typename Scalar = double>
inline void saveTrajectoryTUM(const std::string& filename,
                              const std::vector<Transform_<Scalar>>& trajectory) {
  std::ofstream fs(filename);
  if (!fs.is_open()) {
    throw IOError("Cannot open file for writing: " + filename);
  }

  fs << "# timestamp tx ty tz qx qy qz qw\n";

  for (const auto& tf : trajectory) {
    double timestamp_sec = time::toSec(tf.timestamp());
    auto t = tf.translation();
    auto q = tf.quaternion();

    fs << std::fixed << std::setprecision(9) << timestamp_sec << " "
       << std::setprecision(6) << static_cast<double>(t.x()) << " "
       << static_cast<double>(t.y()) << " " << static_cast<double>(t.z()) << " "
       << static_cast<double>(q.x()) << " " << static_cast<double>(q.y()) << " "
       << static_cast<double>(q.z()) << " " << static_cast<double>(q.w())
       << "\n";
  }
}

// =============================================================================
// Load Transform
// =============================================================================

/**
 * @brief Load a single Transform from TUM trajectory format
 *
 * Reads the first non-comment line from the file.
 *
 * @param filename Input file path
 * @param parent Parent frame name for the loaded Transform
 * @param child Child frame name for the loaded Transform
 * @return Transform with timestamp and pose from file
 * @throws IOError if file cannot be opened or parsed
 *
 * @code
 *   auto tf = io::loadTransformTUM("pose.txt", "map", "base_link");
 * @endcode
 */
template <typename Scalar = double>
[[nodiscard]] inline Transform_<Scalar> loadTransformTUM(
    const std::string& filename, const std::string& parent,
    const std::string& child) {
  std::ifstream fs(filename);
  if (!fs.is_open()) {
    throw IOError("Cannot open file: " + filename);
  }

  std::string line;
  while (std::getline(fs, line)) {
    if (line.empty() || line[0] == '#') continue;

    std::stringstream ss(line);
    double ts, tx, ty, tz, qx, qy, qz, qw;

    if (!(ss >> ts >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {
      throw IOError("Invalid TUM format in: " + filename);
    }

    auto pose = math::isometryFromQuaternion<Scalar>(
        static_cast<Scalar>(qx), static_cast<Scalar>(qy),
        static_cast<Scalar>(qz), static_cast<Scalar>(qw),
        static_cast<Scalar>(tx), static_cast<Scalar>(ty),
        static_cast<Scalar>(tz));

    Transform_<Scalar> tf(parent, child, pose);
    tf.setTimestamp(time::fromSec(ts));
    return tf;
  }

  throw IOError("No valid pose data in: " + filename);
}

/**
 * @brief Load multiple Transforms from TUM trajectory format
 *
 * @param filename Input file path
 * @param parent Parent frame name for all loaded Transforms
 * @param child Child frame name for all loaded Transforms
 * @return Vector of Transforms with timestamps from file
 * @throws IOError if file cannot be opened or contains no valid poses
 *
 * @code
 *   auto trajectory = io::loadTrajectoryTUM("trajectory.txt", "map", "base_link");
 *   for (const auto& tf : trajectory) {
 *     std::cout << "t=" << time::toSec(tf.timestamp()) << "\n";
 *   }
 * @endcode
 */
template <typename Scalar = double>
[[nodiscard]] inline std::vector<Transform_<Scalar>> loadTrajectoryTUM(
    const std::string& filename, const std::string& parent,
    const std::string& child) {
  std::ifstream fs(filename);
  if (!fs.is_open()) {
    throw IOError("Cannot open file: " + filename);
  }

  std::vector<Transform_<Scalar>> trajectory;

  std::string line;
  while (std::getline(fs, line)) {
    if (line.empty() || line[0] == '#') continue;

    std::stringstream ss(line);
    double ts, tx, ty, tz, qx, qy, qz, qw;

    if (!(ss >> ts >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {
      continue;  // Skip invalid lines
    }

    auto pose = math::isometryFromQuaternion<Scalar>(
        static_cast<Scalar>(qx), static_cast<Scalar>(qy),
        static_cast<Scalar>(qz), static_cast<Scalar>(qw),
        static_cast<Scalar>(tx), static_cast<Scalar>(ty),
        static_cast<Scalar>(tz));

    Transform_<Scalar> tf(parent, child, pose);
    tf.setTimestamp(time::fromSec(ts));
    trajectory.push_back(std::move(tf));
  }

  if (trajectory.empty()) {
    throw IOError("No valid poses loaded from: " + filename);
  }

  return trajectory;
}

}  // namespace io
}  // namespace npcl

#endif  // NANOPCL_IO_TRAJECTORY_IO_HPP
