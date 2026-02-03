// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#include <gtest/gtest.h>

#include "fastdem/elevation_map.hpp"

using namespace fastdem;

class ElevationMapTest : public ::testing::Test {
 protected:
  ElevationMap map;

  void SetUp() override { map.setGeometry(10.0f, 10.0f, 0.5f); }
};

TEST_F(ElevationMapTest, IsInitializedAfterSetGeometry) {
  EXPECT_TRUE(map.isInitialized());
}

TEST_F(ElevationMapTest, DefaultConstructorIsNotInitialized) {
  ElevationMap empty_map;
  EXPECT_FALSE(empty_map.isInitialized());
}

TEST_F(ElevationMapTest, IsEmptyAfterSetGeometry) {
  EXPECT_TRUE(map.isEmpty());
}

TEST_F(ElevationMapTest, ElevationAtOutOfBoundsPosition) {
  grid_map::Position far_away(100.0, 100.0);
  EXPECT_TRUE(std::isnan(map.elevationAt(far_away)));
}

TEST_F(ElevationMapTest, HasElevationAtReturnsFalseForUnmeasured) {
  grid_map::Position center(0.0, 0.0);
  EXPECT_FALSE(map.hasElevationAt(center));
}

TEST_F(ElevationMapTest, HasElevationAtReturnsTrueAfterWrite) {
  grid_map::Position pos(1.0, 1.0);
  grid_map::Index idx;
  ASSERT_TRUE(map.getIndex(pos, idx));

  map.at(layer::elevation, idx) = 1.5f;
  EXPECT_TRUE(map.hasElevationAt(pos));
  EXPECT_FLOAT_EQ(map.elevationAt(pos), 1.5f);
}

TEST_F(ElevationMapTest, ClearAtResetsToNaN) {
  grid_map::Position pos(1.0, 1.0);
  grid_map::Index idx;
  ASSERT_TRUE(map.getIndex(pos, idx));

  map.at(layer::elevation, idx) = 2.0f;
  EXPECT_TRUE(map.hasElevationAt(pos));

  map.clearAt(idx);
  EXPECT_FALSE(map.hasElevationAt(pos));
  EXPECT_TRUE(std::isnan(map.elevationAt(pos)));
}

TEST_F(ElevationMapTest, ElevationAtPositionInsideMap) {
  grid_map::Position center(0.0, 0.0);
  grid_map::Index idx;
  ASSERT_TRUE(map.getIndex(center, idx));

  map.at(layer::elevation, idx) = 3.0f;
  EXPECT_TRUE(map.hasElevationAt(center));
  EXPECT_FLOAT_EQ(map.elevationAt(center), 3.0f);
}

TEST_F(ElevationMapTest, ParameterizedConstructor) {
  ElevationMap param_map(5.0f, 5.0f, 1.0f, "world");
  EXPECT_TRUE(param_map.isInitialized());
  EXPECT_TRUE(param_map.isEmpty());
  EXPECT_EQ(param_map.getFrameId(), "world");
}

TEST_F(ElevationMapTest, IsEmptyAtIndex) {
  grid_map::Position pos(2.0, 2.0);
  grid_map::Index idx;
  ASSERT_TRUE(map.getIndex(pos, idx));

  EXPECT_TRUE(map.isEmptyAt(idx));

  map.at(layer::elevation, idx) = 0.5f;
  EXPECT_FALSE(map.isEmptyAt(idx));
}

// ── Index-based API (requires basicLayers = {elevation}) ─────────────────────

TEST_F(ElevationMapTest, HasElevationAtIndexFalseWhenUnmeasured) {
  grid_map::Position pos(1.0, 1.0);
  grid_map::Index idx;
  ASSERT_TRUE(map.getIndex(pos, idx));

  EXPECT_FALSE(map.hasElevationAt(idx));
}

TEST_F(ElevationMapTest, HasElevationAtIndexTrueAfterWrite) {
  grid_map::Position pos(1.0, 1.0);
  grid_map::Index idx;
  ASSERT_TRUE(map.getIndex(pos, idx));

  map.at(layer::elevation, idx) = 1.5f;
  EXPECT_TRUE(map.hasElevationAt(idx));
  EXPECT_FLOAT_EQ(map.elevationAt(idx), 1.5f);
}

TEST_F(ElevationMapTest, ElevationAtIndexReturnsNaNWhenUnmeasured) {
  grid_map::Position pos(0.0, 0.0);
  grid_map::Index idx;
  ASSERT_TRUE(map.getIndex(pos, idx));

  EXPECT_TRUE(std::isnan(map.elevationAt(idx)));
}

TEST_F(ElevationMapTest, ClearAtResetsIndexBasedAccess) {
  grid_map::Position pos(1.0, 1.0);
  grid_map::Index idx;
  ASSERT_TRUE(map.getIndex(pos, idx));

  map.at(layer::elevation, idx) = 2.0f;
  EXPECT_TRUE(map.hasElevationAt(idx));

  map.clearAt(idx);
  EXPECT_FALSE(map.hasElevationAt(idx));
  EXPECT_TRUE(std::isnan(map.elevationAt(idx)));
}

TEST_F(ElevationMapTest, IsNotEmptyAfterSetGeometryOnWholeMap) {
  // Write to all cells, confirm isEmpty returns false
  grid_map::Position center(0.0, 0.0);
  grid_map::Index idx;
  map.getIndex(center, idx);
  map.at(layer::elevation, idx) = 1.0f;

  EXPECT_FALSE(map.isEmpty());
}

TEST_F(ElevationMapTest, DirectDataAccessWorks) {
  // Verify low-level at() always works for reading/writing
  grid_map::Position pos(-2.0, -2.0);
  grid_map::Index idx;
  ASSERT_TRUE(map.getIndex(pos, idx));

  EXPECT_TRUE(std::isnan(map.at(layer::elevation, idx)));
  map.at(layer::elevation, idx) = 42.0f;
  EXPECT_FLOAT_EQ(map.at(layer::elevation, idx), 42.0f);
}
