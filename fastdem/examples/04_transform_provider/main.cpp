// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * 04_transform_provider - Online mode with mock transform systems
 *
 * Demonstrates:
 * - Implementing Calibration and Odometry interfaces
 * - Using online integrate() with automatic transform lookup
 * - Multi-frame integration with simulated robot motion
 */

#include <fastdem/fastdem.hpp>
#include <fastdem/transform_interface.hpp>

#include <iostream>
#include <memory>

#include "../common/data_loader.hpp"
#include "../common/timer.hpp"
#include "../common/visualization.hpp"

using namespace fastdem;

// --- Mock transform providers ---

class MockCalibration : public Calibration {
 public:
  std::optional<Eigen::Isometry3d> getExtrinsic(
      const std::string& sensor_frame) const override {
    if (sensor_frame == "sensor") {
      Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
      T.translation().z() = 1.0;  // sensor mounted 1m above base
      return T;
    }
    return std::nullopt;
  }
  std::string getBaseFrame() const override { return "base_link"; }
};

class MockOdometry : public Odometry {
 public:
  std::optional<Eigen::Isometry3d> getPoseAt(
      uint64_t timestamp_ns) const override {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    double t = static_cast<double>(timestamp_ns) * 1e-9;  // ns -> seconds
    T.translation().x() = t * 0.5;  // 0.5 m/s forward motion
    return T;
  }
  std::string getWorldFrame() const override { return "map"; }
};

// --- Main ---

int main() {
  std::cout << "=== 04_transform_provider ===\n" << std::endl;

  // 1. Create map (larger for robot motion)
  ElevationMap map;
  map.setGeometry(20.0f, 20.0f, 0.1f);
  map.setFrameId("map");

  // 2. Create FastDEM and configure
  FastDEM mapper(map);
  mapper.setHeightFilter(-2.0f, 3.0f)
      .setRangeFilter(0.5f, 10.0f)
      .setEstimatorType(EstimationType::Kalman)
      .setSensorModel(SensorType::Constant);

  // 3. Register mock transform systems
  auto calib = std::make_shared<MockCalibration>();
  auto odom = std::make_shared<MockOdometry>();
  mapper.setCalibrationSystem(calib).setOdometrySystem(odom);

  std::cout << "Transform systems registered" << std::endl;
  std::cout << "  Calibration: sensor -> " << calib->getBaseFrame() << std::endl;
  std::cout << "  Odometry: " << calib->getBaseFrame() << " -> "
            << odom->getWorldFrame() << "\n"
            << std::endl;

  // 4. Multi-frame integration loop
  examples::Timer timer;
  for (int frame = 0; frame < 5; ++frame) {
    auto cloud = std::make_shared<PointCloud>(
        examples::generateTerrainCloud(10000, 4.0f, 42 + frame));
    cloud->setFrameId("sensor");
    cloud->setTimestamp(static_cast<uint64_t>(frame) * 1000000000ULL);  // ns

    timer.start();
    bool ok = mapper.integrate(cloud);
    std::cout << "Frame " << frame << ": " << (ok ? "OK" : "skipped") << " | ";
    timer.printElapsed("time");
  }

  // 5. Results
  examples::printMapStats(map);
  examples::saveMapImage(map,
                         std::string(EXAMPLE_OUTPUT_DIR) + "/output.png");

  return 0;
}
