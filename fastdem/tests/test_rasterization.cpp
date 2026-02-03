// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#include <gtest/gtest.h>

#include "fastdem/fastdem.hpp"
#include "fastdem/mapping/rasterization.hpp"

using namespace fastdem;

class RasterizationTest : public ::testing::Test {
 protected:
  ElevationMap map;

  void SetUp() override { map.setGeometry(10.0f, 10.0f, 1.0f); }
};

TEST_F(RasterizationTest, EmptyCloudReturnsEmpty) {
  Rasterization raster(RasterMethod::Max);
  PointCloud empty;
  auto result = raster.process(empty, map);
  EXPECT_TRUE(result.empty());
}

TEST_F(RasterizationTest, SinglePointPerCell) {
  PointCloud cloud;
  cloud.add(0.0f, 0.0f, 1.0f);
  cloud.add(2.0f, 2.0f, 3.0f);

  Rasterization raster_max(RasterMethod::Max);
  Rasterization raster_min(RasterMethod::Min);
  Rasterization raster_mean(RasterMethod::Mean);

  auto max_result = raster_max.process(cloud, map);
  auto min_result = raster_min.process(cloud, map);
  auto mean_result = raster_mean.process(cloud, map);

  EXPECT_EQ(max_result.size(), 2u);
  EXPECT_EQ(min_result.size(), 2u);
  EXPECT_EQ(mean_result.size(), 2u);
}

TEST_F(RasterizationTest, MaxSelectsHighestZ) {
  PointCloud cloud;
  // Two points in the same cell (within 1m resolution at center)
  cloud.add(0.1f, 0.1f, 1.0f);
  cloud.add(0.2f, 0.2f, 5.0f);
  cloud.add(0.3f, 0.3f, 3.0f);

  Rasterization raster(RasterMethod::Max);
  auto result = raster.process(cloud, map);

  ASSERT_EQ(result.size(), 1u);
  EXPECT_FLOAT_EQ(result.point(0).z(), 5.0f);
}

TEST_F(RasterizationTest, MinSelectsLowestZ) {
  PointCloud cloud;
  cloud.add(0.1f, 0.1f, 1.0f);
  cloud.add(0.2f, 0.2f, 5.0f);
  cloud.add(0.3f, 0.3f, 3.0f);

  Rasterization raster(RasterMethod::Min);
  auto result = raster.process(cloud, map);

  ASSERT_EQ(result.size(), 1u);
  EXPECT_FLOAT_EQ(result.point(0).z(), 1.0f);
}

TEST_F(RasterizationTest, MeanComputesAverageZ) {
  PointCloud cloud;
  cloud.add(0.1f, 0.1f, 1.0f);
  cloud.add(0.2f, 0.2f, 5.0f);

  Rasterization raster(RasterMethod::Mean);
  auto result = raster.process(cloud, map);

  ASSERT_EQ(result.size(), 1u);
  EXPECT_FLOAT_EQ(result.point(0).z(), 3.0f);
}

TEST_F(RasterizationTest, OutOfBoundsPointsIgnored) {
  PointCloud cloud;
  cloud.add(100.0f, 100.0f, 1.0f);  // Way outside 10x10 map
  cloud.add(-100.0f, -100.0f, 2.0f);

  Rasterization raster(RasterMethod::Max);
  auto result = raster.process(cloud, map);

  EXPECT_TRUE(result.empty());
}

TEST_F(RasterizationTest, IntensityChannelPreserved) {
  PointCloud cloud;
  cloud.add(0.1f, 0.1f, 1.0f, nanopcl::Intensity(0.5f));
  cloud.add(0.2f, 0.2f, 5.0f, nanopcl::Intensity(0.8f));

  // Max: should pick point with z=5.0 (intensity=0.8)
  Rasterization raster_max(RasterMethod::Max);
  auto max_result = raster_max.process(cloud, map);
  ASSERT_EQ(max_result.size(), 1u);
  ASSERT_TRUE(max_result.hasIntensity());
  EXPECT_FLOAT_EQ(max_result.intensity(0), 0.8f);

  // Mean: should average intensities
  Rasterization raster_mean(RasterMethod::Mean);
  auto mean_result = raster_mean.process(cloud, map);
  ASSERT_EQ(mean_result.size(), 1u);
  ASSERT_TRUE(mean_result.hasIntensity());
  EXPECT_FLOAT_EQ(mean_result.intensity(0), 0.65f);
}

TEST_F(RasterizationTest, CovarianceChannelPreserved) {
  PointCloud cloud;
  cloud.useCovariance();
  cloud.add(0.1f, 0.1f, 2.0f);
  cloud.covariance(0) = Eigen::Matrix3f::Identity() * 0.01f;

  cloud.add(0.2f, 0.2f, 4.0f);
  cloud.covariance(1) = Eigen::Matrix3f::Identity() * 0.03f;

  // Mean: should average covariances
  Rasterization raster(RasterMethod::Mean);
  auto result = raster.process(cloud, map);
  ASSERT_EQ(result.size(), 1u);
  ASSERT_TRUE(result.hasCovariance());
  EXPECT_NEAR(result.covariance(0)(0, 0), 0.02f, 1e-6f);
}

// ─── rasterize tests ────────────────────────────────────────────────────

TEST(RasterizeTest, EmptyCloudIsNoOp) {
  ElevationMap map(10.0f, 10.0f, 1.0f, "map");
  PointCloud empty;
  rasterize(empty, map);
  EXPECT_TRUE(map.isEmpty());
}

TEST(RasterizeTest, WritesElevationToMap) {
  ElevationMap map(10.0f, 10.0f, 1.0f, "map");
  PointCloud cloud;
  cloud.add(0.0f, 0.0f, 2.5f);
  cloud.add(1.5f, 1.5f, 3.0f);

  rasterize(cloud, map);

  EXPECT_TRUE(map.hasElevationAt(grid_map::Position(0.0, 0.0)));
  EXPECT_FLOAT_EQ(map.elevationAt(grid_map::Position(0.0, 0.0)), 2.5f);
  EXPECT_TRUE(map.hasElevationAt(grid_map::Position(1.5, 1.5)));
  EXPECT_FLOAT_EQ(map.elevationAt(grid_map::Position(1.5, 1.5)), 3.0f);
}

TEST(RasterizeTest, MaxSelectsHighestInCell) {
  ElevationMap map(10.0f, 10.0f, 1.0f, "map");
  PointCloud cloud;
  cloud.add(0.1f, 0.1f, 1.0f);
  cloud.add(0.2f, 0.2f, 5.0f);
  cloud.add(0.3f, 0.3f, 3.0f);

  rasterize(cloud, map, RasterMethod::Max);

  EXPECT_FLOAT_EQ(map.elevationAt(grid_map::Position(0.2, 0.2)), 5.0f);
}

TEST(RasterizeTest, IntensityWrittenWhenLayerExists) {
  ElevationMap map(10.0f, 10.0f, 1.0f, "map");
  map.add(layer::intensity, NAN);

  PointCloud cloud;
  cloud.add(0.0f, 0.0f, 1.0f, nanopcl::Intensity(0.7f));

  rasterize(cloud, map);

  grid_map::Index index;
  map.getIndex(grid_map::Position(0.0, 0.0), index);
  EXPECT_FLOAT_EQ(map.at(layer::intensity, index), 0.7f);
}

TEST(RasterizeTest, AutoSizedMapFitsCloud) {
  PointCloud cloud;
  cloud.add(-5.0f, -3.0f, 1.0f);
  cloud.add(5.0f, 3.0f, 2.0f);
  cloud.add(0.0f, 0.0f, 1.5f);

  auto map = rasterize(cloud, 0.5f);

  // All points should be inside the map
  EXPECT_TRUE(map.isInside(grid_map::Position(-5.0, -3.0)));
  EXPECT_TRUE(map.isInside(grid_map::Position(5.0, 3.0)));
  EXPECT_TRUE(map.isInside(grid_map::Position(0.0, 0.0)));

  // All points should have elevation
  EXPECT_TRUE(map.hasElevationAt(grid_map::Position(-5.0, -3.0)));
  EXPECT_TRUE(map.hasElevationAt(grid_map::Position(5.0, 3.0)));
  EXPECT_TRUE(map.hasElevationAt(grid_map::Position(0.0, 0.0)));
}

TEST(RasterizeTest, AutoSizedEmptyCloudReturnsEmptyMap) {
  PointCloud empty;
  auto map = rasterize(empty, 0.5f);
  EXPECT_FALSE(map.isInitialized());
}
