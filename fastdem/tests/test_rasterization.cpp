// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#include <gtest/gtest.h>

#include "fastdem/io/pcd_convert.hpp"

using namespace fastdem;

// ─── fromPointCloud tests ───────────────────────────────────────────────

TEST(FromPointCloudTest, EmptyCloudIsNoOp) {
  ElevationMap map(10.0f, 10.0f, 1.0f, "map");
  PointCloud empty;
  fromPointCloud(empty, map);
  EXPECT_TRUE(map.isEmpty());
}

TEST(FromPointCloudTest, WritesElevationToMap) {
  ElevationMap map(10.0f, 10.0f, 1.0f, "map");
  PointCloud cloud;
  cloud.add(0.0f, 0.0f, 2.5f);
  cloud.add(1.5f, 1.5f, 3.0f);

  fromPointCloud(cloud, map);

  EXPECT_TRUE(map.hasElevationAt(grid_map::Position(0.0, 0.0)));
  EXPECT_FLOAT_EQ(map.elevationAt(grid_map::Position(0.0, 0.0)), 2.5f);
  EXPECT_TRUE(map.hasElevationAt(grid_map::Position(1.5, 1.5)));
  EXPECT_FLOAT_EQ(map.elevationAt(grid_map::Position(1.5, 1.5)), 3.0f);
}

TEST(FromPointCloudTest, MaxSelectsHighestInCell) {
  ElevationMap map(10.0f, 10.0f, 1.0f, "map");
  PointCloud cloud;
  cloud.add(0.1f, 0.1f, 1.0f);
  cloud.add(0.2f, 0.2f, 5.0f);
  cloud.add(0.3f, 0.3f, 3.0f);

  fromPointCloud(cloud, map, RasterMethod::Max);

  EXPECT_FLOAT_EQ(map.elevationAt(grid_map::Position(0.2, 0.2)), 5.0f);
}

TEST(FromPointCloudTest, IntensityWrittenWhenLayerExists) {
  ElevationMap map(10.0f, 10.0f, 1.0f, "map");
  map.add(layer::intensity, NAN);

  PointCloud cloud;
  cloud.add(0.0f, 0.0f, 1.0f, nanopcl::Intensity(0.7f));

  fromPointCloud(cloud, map);

  grid_map::Index index;
  map.getIndex(grid_map::Position(0.0, 0.0), index);
  EXPECT_FLOAT_EQ(map.at(layer::intensity, index), 0.7f);
}

TEST(FromPointCloudTest, AutoSizedMapFitsCloud) {
  PointCloud cloud;
  cloud.add(-5.0f, -3.0f, 1.0f);
  cloud.add(5.0f, 3.0f, 2.0f);
  cloud.add(0.0f, 0.0f, 1.5f);

  auto map = fromPointCloud(cloud, 0.5f);

  // All points should be inside the map
  EXPECT_TRUE(map.isInside(grid_map::Position(-5.0, -3.0)));
  EXPECT_TRUE(map.isInside(grid_map::Position(5.0, 3.0)));
  EXPECT_TRUE(map.isInside(grid_map::Position(0.0, 0.0)));

  // All points should have elevation
  EXPECT_TRUE(map.hasElevationAt(grid_map::Position(-5.0, -3.0)));
  EXPECT_TRUE(map.hasElevationAt(grid_map::Position(5.0, 3.0)));
  EXPECT_TRUE(map.hasElevationAt(grid_map::Position(0.0, 0.0)));
}

TEST(FromPointCloudTest, AutoSizedEmptyCloudReturnsEmptyMap) {
  PointCloud empty;
  auto map = fromPointCloud(empty, 0.5f);
  EXPECT_FALSE(map.isInitialized());
}

TEST(FromPointCloudTest, MinSelectsLowestInCell) {
  ElevationMap map(10.0f, 10.0f, 1.0f, "map");
  PointCloud cloud;
  cloud.add(0.1f, 0.1f, 1.0f);
  cloud.add(0.2f, 0.2f, 5.0f);
  cloud.add(0.3f, 0.3f, 3.0f);

  fromPointCloud(cloud, map, RasterMethod::Min);

  EXPECT_FLOAT_EQ(map.elevationAt(grid_map::Position(0.2, 0.2)), 1.0f);
}

TEST(FromPointCloudTest, MeanComputesAverageInCell) {
  ElevationMap map(10.0f, 10.0f, 1.0f, "map");
  PointCloud cloud;
  cloud.add(0.1f, 0.1f, 2.0f);
  cloud.add(0.2f, 0.2f, 4.0f);

  fromPointCloud(cloud, map, RasterMethod::Mean);

  EXPECT_FLOAT_EQ(map.elevationAt(grid_map::Position(0.1, 0.1)), 3.0f);
}

TEST(FromPointCloudTest, ColorWrittenWhenLayerExists) {
  ElevationMap map(10.0f, 10.0f, 1.0f, "map");
  map.add(layer::color, NAN);

  PointCloud cloud;
  cloud.add(0.0f, 0.0f, 1.0f, nanopcl::Color{255, 128, 64});

  fromPointCloud(cloud, map);

  grid_map::Index index;
  map.getIndex(grid_map::Position(0.0, 0.0), index);
  float packed = map.at(layer::color, index);
  EXPECT_FALSE(std::isnan(packed));

  // colorValueToVector(float, Vector3f) returns [0, 1] range
  Eigen::Vector3f rgb;
  grid_map::colorValueToVector(packed, rgb);
  EXPECT_NEAR(rgb(0), 1.0f, 1.0f / 255.0f);           // R: 255
  EXPECT_NEAR(rgb(1), 128.0f / 255.0f, 1.0f / 255.0f); // G: 128
  EXPECT_NEAR(rgb(2), 64.0f / 255.0f, 1.0f / 255.0f);  // B: 64
}

TEST(FromPointCloudTest, AutoSizedMapDimensions) {
  PointCloud cloud;
  cloud.add(-5.0f, -3.0f, 1.0f);
  cloud.add(5.0f, 3.0f, 2.0f);

  const float resolution = 0.5f;
  auto map = fromPointCloud(cloud, resolution);

  // Expected: (max - min + resolution) = bounding box + one cell margin
  const auto& length = map.getLength();
  EXPECT_NEAR(length(0), 10.0f + resolution, resolution);  // x: 10.5m
  EXPECT_NEAR(length(1), 6.0f + resolution, resolution);   // y: 6.5m
  EXPECT_FLOAT_EQ(map.getResolution(), resolution);
}

// ─── fromPointCloud() statistics tests ─────────────────────────────────

TEST(FromPointCloudStatsTest, CreatesStatisticsLayers) {
  ElevationMap map(10.0f, 10.0f, 1.0f, "map");
  PointCloud cloud;
  cloud.add(0.0f, 0.0f, 1.0f);

  fromPointCloud(cloud, map);

  EXPECT_TRUE(map.exists(layer::elevation_min));
  EXPECT_TRUE(map.exists(layer::elevation_max));
  EXPECT_TRUE(map.exists(layer::variance));
  EXPECT_TRUE(map.exists(layer::n_points));
}

TEST(FromPointCloudStatsTest, MinMaxCountCorrect) {
  ElevationMap map(10.0f, 10.0f, 1.0f, "map");
  PointCloud cloud;
  cloud.add(0.1f, 0.1f, 2.0f);
  cloud.add(0.2f, 0.2f, 6.0f);
  cloud.add(0.3f, 0.3f, 4.0f);

  fromPointCloud(cloud, map, RasterMethod::Max);

  grid_map::Index index;
  ASSERT_TRUE(map.getIndex(grid_map::Position(0.2, 0.2), index));

  EXPECT_FLOAT_EQ(map.at(layer::elevation_min, index), 2.0f);
  EXPECT_FLOAT_EQ(map.at(layer::elevation_max, index), 6.0f);
  EXPECT_FLOAT_EQ(map.at(layer::n_points, index), 3.0f);
}

TEST(FromPointCloudStatsTest, VarianceIsWelfordSampleVariance) {
  ElevationMap map(10.0f, 10.0f, 1.0f, "map");
  PointCloud cloud;
  // z = {2, 4, 6} → mean = 4, sample_variance = ((2-4)²+(4-4)²+(6-4)²)/(3-1)
  // = 4.0
  cloud.add(0.1f, 0.1f, 2.0f);
  cloud.add(0.2f, 0.2f, 4.0f);
  cloud.add(0.3f, 0.3f, 6.0f);

  fromPointCloud(cloud, map, RasterMethod::Mean);

  grid_map::Index index;
  ASSERT_TRUE(map.getIndex(grid_map::Position(0.2, 0.2), index));

  EXPECT_NEAR(map.at(layer::variance, index), 4.0f, 1e-5f);
}

TEST(FromPointCloudStatsTest, SinglePointVarianceIsZero) {
  ElevationMap map(10.0f, 10.0f, 1.0f, "map");
  PointCloud cloud;
  cloud.add(0.0f, 0.0f, 3.0f);

  fromPointCloud(cloud, map);

  grid_map::Index index;
  ASSERT_TRUE(map.getIndex(grid_map::Position(0.0, 0.0), index));

  EXPECT_FLOAT_EQ(map.at(layer::variance, index), 0.0f);
  EXPECT_FLOAT_EQ(map.at(layer::n_points, index), 1.0f);
}

TEST(FromPointCloudStatsTest, MinMethodWritesMinElevation) {
  ElevationMap map(10.0f, 10.0f, 1.0f, "map");
  PointCloud cloud;
  cloud.add(0.1f, 0.1f, 1.0f);
  cloud.add(0.2f, 0.2f, 5.0f);
  cloud.add(0.3f, 0.3f, 3.0f);

  fromPointCloud(cloud, map, RasterMethod::Min);

  grid_map::Index index;
  ASSERT_TRUE(map.getIndex(grid_map::Position(0.2, 0.2), index));

  // elevation = min_z
  EXPECT_FLOAT_EQ(map.at(layer::elevation, index), 1.0f);
  // elevation_max still has the true max
  EXPECT_FLOAT_EQ(map.at(layer::elevation_max, index), 5.0f);
}

TEST(FromPointCloudStatsTest, MeanMethodWritesMeanElevation) {
  ElevationMap map(10.0f, 10.0f, 1.0f, "map");
  PointCloud cloud;
  cloud.add(0.1f, 0.1f, 2.0f);
  cloud.add(0.2f, 0.2f, 4.0f);
  cloud.add(0.3f, 0.3f, 6.0f);

  fromPointCloud(cloud, map, RasterMethod::Mean);

  grid_map::Index index;
  ASSERT_TRUE(map.getIndex(grid_map::Position(0.2, 0.2), index));

  EXPECT_FLOAT_EQ(map.at(layer::elevation, index), 4.0f);
}

TEST(FromPointCloudStatsTest, AutoSizedMapHasStatistics) {
  PointCloud cloud;
  cloud.add(0.0f, 0.0f, 1.0f);
  cloud.add(0.1f, 0.1f, 3.0f);

  auto map = fromPointCloud(cloud, 1.0f);

  EXPECT_TRUE(map.exists(layer::elevation_min));
  EXPECT_TRUE(map.exists(layer::elevation_max));
  EXPECT_TRUE(map.exists(layer::variance));
  EXPECT_TRUE(map.exists(layer::n_points));
}

TEST(FromPointCloudStatsTest, IntensityLayerAutoCreated) {
  ElevationMap map(10.0f, 10.0f, 1.0f, "map");
  // Do NOT pre-add intensity layer — fromPointCloud should auto-create it
  PointCloud cloud;
  cloud.add(0.0f, 0.0f, 1.0f, nanopcl::Intensity(0.7f));

  fromPointCloud(cloud, map);

  EXPECT_TRUE(map.exists(layer::intensity));
  grid_map::Index index;
  ASSERT_TRUE(map.getIndex(grid_map::Position(0.0, 0.0), index));
  EXPECT_FLOAT_EQ(map.at(layer::intensity, index), 0.7f);
}

TEST(FromPointCloudStatsTest, NaNPointsSkipped) {
  ElevationMap map(10.0f, 10.0f, 1.0f, "map");
  PointCloud cloud;
  // Same cell: one NaN, two valid
  cloud.add(0.0f, 0.0f, std::numeric_limits<float>::quiet_NaN());
  cloud.add(0.0f, 0.0f, 2.0f);
  cloud.add(0.0f, 0.0f, 4.0f);

  fromPointCloud(cloud, map);

  grid_map::Index index;
  ASSERT_TRUE(map.getIndex(grid_map::Position(0.0, 0.0), index));

  // Only the two non-NaN points should be counted
  EXPECT_FLOAT_EQ(map.at(layer::n_points, index), 2.0f);
  EXPECT_FLOAT_EQ(map.at(layer::elevation, index), 4.0f);  // Max method
  EXPECT_FLOAT_EQ(map.at(layer::elevation_min, index), 2.0f);
}

// ─── toPointCloud tests ────────────────────────────────────────────────

TEST(ToPointCloudTest, EmptyMapReturnsEmptyCloud) {
  ElevationMap map(10.0f, 10.0f, 1.0f, "map");
  auto cloud = toPointCloud(map);
  EXPECT_TRUE(cloud.empty());
}

TEST(ToPointCloudTest, ValidCellsConverted) {
  ElevationMap map(10.0f, 10.0f, 1.0f, "map");
  PointCloud input;
  input.add(0.0f, 0.0f, 1.5f);
  input.add(2.0f, 2.0f, 3.0f);
  fromPointCloud(input, map);

  auto cloud = toPointCloud(map);
  EXPECT_EQ(cloud.size(), 2u);
}

TEST(ToPointCloudTest, ElevationPreserved) {
  PointCloud input;
  input.add(0.0f, 0.0f, 2.5f);

  auto map = fromPointCloud(input, 1.0f);
  auto cloud = toPointCloud(map);

  ASSERT_EQ(cloud.size(), 1u);
  EXPECT_FLOAT_EQ(cloud.point(0).z(), 2.5f);
}

TEST(ToPointCloudTest, IntensityPreserved) {
  PointCloud input;
  input.add(0.0f, 0.0f, 1.0f, nanopcl::Intensity(0.8f));

  auto map = fromPointCloud(input, 1.0f);
  auto cloud = toPointCloud(map);

  ASSERT_EQ(cloud.size(), 1u);
  ASSERT_TRUE(cloud.hasIntensity());
  EXPECT_FLOAT_EQ(cloud.intensity(0), 0.8f);
}

TEST(ToPointCloudTest, RoundTripPreservesCount) {
  PointCloud input;
  // Place points in different cells (spacing > resolution)
  input.add(0.0f, 0.0f, 1.0f);
  input.add(2.0f, 0.0f, 2.0f);
  input.add(0.0f, 2.0f, 3.0f);
  input.add(2.0f, 2.0f, 4.0f);

  auto map = fromPointCloud(input, 1.0f);
  auto cloud = toPointCloud(map);

  EXPECT_EQ(cloud.size(), 4u);
}

// ─── buildDEM tests ────────────────────────────────────────────────────

TEST(BuildDEMTest, EmptyCloudReturnsUninitialized) {
  PointCloud empty;
  auto map = buildDEM(empty);
  EXPECT_FALSE(map.isInitialized());
}

TEST(BuildDEMTest, BasicPipeline) {
  // Create a simple ground plane with some noise
  PointCloud cloud;
  for (float x = -2.0f; x <= 2.0f; x += 0.1f) {
    for (float y = -2.0f; y <= 2.0f; y += 0.1f) {
      cloud.add(x, y, 0.0f);
    }
  }

  DEMConfig config;
  config.resolution = 0.5f;
  config.sor_k = 5;
  config.inpaint_iterations = 0;  // Disable inpainting for determinism

  auto map = buildDEM(cloud, config);

  EXPECT_TRUE(map.isInitialized());
  EXPECT_TRUE(map.exists(layer::elevation));
  EXPECT_TRUE(map.hasElevationAt(grid_map::Position(0.0, 0.0)));
  EXPECT_NEAR(map.elevationAt(grid_map::Position(0.0, 0.0)), 0.0f, 0.1f);
}

TEST(BuildDEMTest, InpaintingFillsHoles) {
  // Create a grid with a gap in the middle
  PointCloud cloud;
  for (float x = -2.0f; x <= 2.0f; x += 0.1f) {
    for (float y = -2.0f; y <= 2.0f; y += 0.1f) {
      if (std::abs(x) < 0.3f && std::abs(y) < 0.3f) continue;  // gap
      cloud.add(x, y, 1.0f);
    }
  }

  DEMConfig config;
  config.resolution = 0.5f;
  config.sor_k = 5;
  config.inpaint_iterations = 3;

  auto map = buildDEM(cloud, config);

  EXPECT_TRUE(map.isInitialized());
  // The center hole should be filled by inpainting
  EXPECT_TRUE(map.hasElevationAt(grid_map::Position(0.0, 0.0)));
}

TEST(BuildDEMTest, ResolutionApplied) {
  PointCloud cloud;
  cloud.add(0.0f, 0.0f, 1.0f);
  cloud.add(1.0f, 1.0f, 2.0f);

  DEMConfig config;
  config.resolution = 0.25f;
  config.sor_k = 1;
  config.inpaint_iterations = 0;

  auto map = buildDEM(cloud, config);

  EXPECT_FLOAT_EQ(map.getResolution(), 0.25f);
}

TEST(BuildDEMTest, OutputHasStatisticsLayers) {
  PointCloud cloud;
  for (float x = -1.0f; x <= 1.0f; x += 0.2f) {
    for (float y = -1.0f; y <= 1.0f; y += 0.2f) {
      cloud.add(x, y, 0.5f);
    }
  }

  DEMConfig config;
  config.resolution = 0.5f;
  config.sor_k = 3;
  config.inpaint_iterations = 0;

  auto map = buildDEM(cloud, config);

  EXPECT_TRUE(map.exists(layer::elevation));
  EXPECT_TRUE(map.exists(layer::elevation_min));
  EXPECT_TRUE(map.exists(layer::elevation_max));
  EXPECT_TRUE(map.exists(layer::variance));
  EXPECT_TRUE(map.exists(layer::n_points));
}
