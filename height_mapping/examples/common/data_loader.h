/*
 * data_loader.h
 *
 * Sample data generation utilities for examples.
 */

#ifndef EXAMPLES_COMMON_DATA_LOADER_H
#define EXAMPLES_COMMON_DATA_LOADER_H

#include <cmath>
#include <nanopcl/core.hpp>
#include <random>

namespace examples {

/**
 * @brief Generate a synthetic point cloud simulating terrain.
 *
 * Creates a point cloud with points distributed over a terrain surface.
 * The terrain is a sinusoidal surface with added noise.
 *
 * @param num_points Number of points to generate
 * @param extent Spatial extent in x and y (default 10.0 meters)
 * @param seed Random seed for reproducibility
 * @return Generated point cloud
 */
inline npcl::PointCloud generateTerrainCloud(size_t num_points,
                                             float extent = 10.0f,
                                             unsigned seed = 42) {
  npcl::PointCloud cloud;
  cloud.reserve(num_points);

  std::mt19937 gen(seed);
  std::uniform_real_distribution<float> xy_dist(-extent / 2, extent / 2);
  std::normal_distribution<float> noise_dist(0.0f, 0.02f);

  for (size_t i = 0; i < num_points; ++i) {
    float x = xy_dist(gen);
    float y = xy_dist(gen);

    // Sinusoidal terrain with noise
    float z = 0.3f * std::sin(x * 0.5f) * std::cos(y * 0.5f) + noise_dist(gen);

    cloud.add(npcl::Point(x, y, z));
  }

  return cloud;
}

/**
 * @brief Generate a flat ground plane with obstacles.
 *
 * Creates a point cloud with a flat ground plane (z=0) and
 * some box-shaped obstacles.
 *
 * @param num_ground_points Number of ground points
 * @param num_obstacle_points Number of obstacle points
 * @param extent Spatial extent in x and y
 * @param seed Random seed
 * @return Generated point cloud
 */
inline npcl::PointCloud generateGroundWithObstacles(size_t num_ground_points,
                                                    size_t num_obstacle_points,
                                                    float extent = 10.0f,
                                                    unsigned seed = 42) {
  npcl::PointCloud cloud;
  cloud.reserve(num_ground_points + num_obstacle_points);

  std::mt19937 gen(seed);
  std::uniform_real_distribution<float> xy_dist(-extent / 2, extent / 2);
  std::normal_distribution<float> ground_noise(0.0f, 0.01f);

  // Ground plane
  for (size_t i = 0; i < num_ground_points; ++i) {
    float x = xy_dist(gen);
    float y = xy_dist(gen);
    float z = ground_noise(gen);
    cloud.add(npcl::Point(x, y, z));
  }

  // Obstacles (two boxes)
  std::uniform_real_distribution<float> box1_x(1.0f, 2.0f);
  std::uniform_real_distribution<float> box1_y(1.0f, 2.0f);
  std::uniform_real_distribution<float> box1_z(0.0f, 0.5f);

  std::uniform_real_distribution<float> box2_x(-3.0f, -2.0f);
  std::uniform_real_distribution<float> box2_y(-1.0f, 0.0f);
  std::uniform_real_distribution<float> box2_z(0.0f, 0.8f);

  for (size_t i = 0; i < num_obstacle_points / 2; ++i) {
    cloud.add(npcl::Point(box1_x(gen), box1_y(gen), box1_z(gen)));
  }
  for (size_t i = 0; i < num_obstacle_points / 2; ++i) {
    cloud.add(npcl::Point(box2_x(gen), box2_y(gen), box2_z(gen)));
  }

  return cloud;
}

/**
 * @brief Generate a simple ramp/slope terrain.
 *
 * @param num_points Number of points
 * @param extent Spatial extent
 * @param slope Slope angle in degrees
 * @param seed Random seed
 * @return Generated point cloud
 */
inline npcl::PointCloud generateRampCloud(size_t num_points,
                                          float extent = 10.0f,
                                          float slope = 15.0f,
                                          unsigned seed = 42) {
  npcl::PointCloud cloud;
  cloud.reserve(num_points);

  std::mt19937 gen(seed);
  std::uniform_real_distribution<float> xy_dist(-extent / 2, extent / 2);
  std::normal_distribution<float> noise_dist(0.0f, 0.01f);

  float slope_rad = slope * M_PI / 180.0f;

  for (size_t i = 0; i < num_points; ++i) {
    float x = xy_dist(gen);
    float y = xy_dist(gen);
    float z = x * std::tan(slope_rad) + noise_dist(gen);
    cloud.add(npcl::Point(x, y, z));
  }

  return cloud;
}

}  // namespace examples

#endif  // EXAMPLES_COMMON_DATA_LOADER_H
