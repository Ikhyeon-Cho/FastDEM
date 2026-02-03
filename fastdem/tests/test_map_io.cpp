// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

#include "fastdem/elevation_map.hpp"
#include "fastdem/io/npz.hpp"
#include "fastdem/io/png.hpp"

using namespace fastdem;

// ─── NPZ tests ──────────────────────────────────────────────────────────────

class NpzTest : public ::testing::Test {
 protected:
  ElevationMap map;
  std::string tmp_path;

  void SetUp() override {
    map = ElevationMap(10.0f, 8.0f, 0.5f, "map");

    auto& elev = map.get(layer::elevation);
    for (int i = 0; i < elev.size(); ++i) {
      elev.data()[i] = static_cast<float>(i) * 0.1f;
    }
    elev(0, 0) = NAN;
    elev(1, 1) = NAN;

    tmp_path = testing::TempDir() + "/test_io.npz";
  }

  void TearDown() override { std::remove(tmp_path.c_str()); }
};

TEST_F(NpzTest, RoundTrip) {
  ASSERT_TRUE(io::saveNpz(tmp_path, map));

  ElevationMap loaded;
  ASSERT_TRUE(io::loadNpz(tmp_path, loaded));

  EXPECT_FLOAT_EQ(loaded.getResolution(), map.getResolution());
  EXPECT_EQ(loaded.getSize()(0), map.getSize()(0));
  EXPECT_EQ(loaded.getSize()(1), map.getSize()(1));
  EXPECT_EQ(loaded.getFrameId(), "map");

  auto pos_orig = map.getPosition();
  auto pos_load = loaded.getPosition();
  EXPECT_NEAR(pos_load.x(), pos_orig.x(), 1e-4);
  EXPECT_NEAR(pos_load.y(), pos_orig.y(), 1e-4);

  ASSERT_TRUE(loaded.exists(layer::elevation));
  const auto& orig = map.get(layer::elevation);
  const auto& load = loaded.get(layer::elevation);
  ASSERT_EQ(orig.rows(), load.rows());
  ASSERT_EQ(orig.cols(), load.cols());

  for (int i = 0; i < orig.size(); ++i) {
    if (std::isnan(orig.data()[i])) {
      EXPECT_TRUE(std::isnan(load.data()[i])) << "index " << i;
    } else {
      EXPECT_FLOAT_EQ(orig.data()[i], load.data()[i]) << "index " << i;
    }
  }
}

TEST_F(NpzTest, RoundTripWithStartIndex) {
  map.setStartIndex(grid_map::Index(5, 3));

  ASSERT_TRUE(io::saveNpz(tmp_path, map));

  ElevationMap loaded;
  ASSERT_TRUE(io::loadNpz(tmp_path, loaded));

  auto start_orig = map.getStartIndex();
  auto start_load = loaded.getStartIndex();
  EXPECT_EQ(start_load(0), start_orig(0));
  EXPECT_EQ(start_load(1), start_orig(1));
}

TEST_F(NpzTest, RoundTripMultipleLayers) {
  map.add("variance");
  auto& var = map.get("variance");
  for (int i = 0; i < var.size(); ++i) {
    var.data()[i] = static_cast<float>(i) * 0.01f;
  }

  ASSERT_TRUE(io::saveNpz(tmp_path, map));

  ElevationMap loaded;
  ASSERT_TRUE(io::loadNpz(tmp_path, loaded));

  ASSERT_TRUE(loaded.exists(layer::elevation));
  ASSERT_TRUE(loaded.exists("variance"));

  const auto& var_orig = map.get("variance");
  const auto& var_load = loaded.get("variance");
  for (int i = 0; i < var_orig.size(); ++i) {
    EXPECT_FLOAT_EQ(var_orig.data()[i], var_load.data()[i]) << "index " << i;
  }
}

TEST_F(NpzTest, SelectiveSave) {
  map.add("variance");
  map.add("intensity");

  ASSERT_TRUE(io::saveNpz(tmp_path, map, {layer::elevation, "variance"}));

  ElevationMap loaded;
  ASSERT_TRUE(io::loadNpz(tmp_path, loaded));

  EXPECT_TRUE(loaded.exists(layer::elevation));
  EXPECT_TRUE(loaded.exists("variance"));
  EXPECT_FALSE(loaded.exists("intensity"));
}

TEST_F(NpzTest, NonexistentLayerSkipped) {
  // Requesting a nonexistent layer should still succeed (skipped with warning)
  ASSERT_TRUE(io::saveNpz(tmp_path, map, {layer::elevation, "no_such_layer"}));

  ElevationMap loaded;
  ASSERT_TRUE(io::loadNpz(tmp_path, loaded));

  EXPECT_TRUE(loaded.exists(layer::elevation));
  EXPECT_FALSE(loaded.exists("no_such_layer"));
}

TEST_F(NpzTest, EmptyMap) {
  // All-NaN map should round-trip correctly
  ElevationMap empty(4.0f, 4.0f, 0.5f, "empty");

  ASSERT_TRUE(io::saveNpz(tmp_path, empty));

  ElevationMap loaded;
  ASSERT_TRUE(io::loadNpz(tmp_path, loaded));

  EXPECT_FLOAT_EQ(loaded.getResolution(), empty.getResolution());
  EXPECT_EQ(loaded.getSize()(0), empty.getSize()(0));

  const auto& elev = loaded.get(layer::elevation);
  for (int i = 0; i < elev.size(); ++i) {
    EXPECT_TRUE(std::isnan(elev.data()[i])) << "index " << i;
  }
}

TEST_F(NpzTest, FutureVersionRejected) {
  // Save a valid file, then patch the version number to a future value
  ASSERT_TRUE(io::saveNpz(tmp_path, map));

  // Read file into memory
  std::ifstream ifs(tmp_path, std::ios::binary);
  std::string content((std::istreambuf_iterator<char>(ifs)),
                      std::istreambuf_iterator<char>());
  ifs.close();

  // Replace "version": 1 with "version": 99
  auto pos = content.find("\"version\": 1");
  ASSERT_NE(pos, std::string::npos);
  content.replace(pos, 12, "\"version\":99");

  std::ofstream ofs(tmp_path, std::ios::binary);
  ofs.write(content.data(), content.size());
  ofs.close();

  ElevationMap loaded;
  EXPECT_FALSE(io::loadNpz(tmp_path, loaded));
}

TEST_F(NpzTest, LoadNonExistentFile) {
  ElevationMap loaded;
  EXPECT_FALSE(io::loadNpz("/tmp/does_not_exist.npz", loaded));
}

TEST_F(NpzTest, SaveToInvalidPath) {
  EXPECT_FALSE(io::saveNpz("/nonexistent/dir/test.npz", map));
}

// ─── PNG tests ──────────────────────────────────────────────────────────────

class PngTest : public ::testing::Test {
 protected:
  ElevationMap map;
  std::string tmp_path;

  void SetUp() override {
    map = ElevationMap(4.0f, 4.0f, 0.5f, "map");

    auto& elev = map.get(layer::elevation);
    for (int i = 0; i < elev.size(); ++i) {
      elev.data()[i] = static_cast<float>(i) * 0.1f;
    }
    elev(0, 0) = NAN;

    tmp_path = testing::TempDir() + "/test_io.png";
  }

  void TearDown() override { std::remove(tmp_path.c_str()); }

  long fileSize(const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    return f.is_open() ? static_cast<long>(f.tellg()) : -1;
  }
};

TEST_F(PngTest, BasicSave) {
  ASSERT_TRUE(io::savePng(tmp_path, map, layer::elevation));
  EXPECT_GT(fileSize(tmp_path), 0);
}

TEST_F(PngTest, ColormapViridis) {
  io::PngExportConfig cfg;
  cfg.colormap = io::PngExportConfig::Colormap::VIRIDIS;
  ASSERT_TRUE(io::savePng(tmp_path, map, layer::elevation, cfg));
  EXPECT_GT(fileSize(tmp_path), 0);
}

TEST_F(PngTest, ColormapJet) {
  io::PngExportConfig cfg;
  cfg.colormap = io::PngExportConfig::Colormap::JET;
  ASSERT_TRUE(io::savePng(tmp_path, map, layer::elevation, cfg));
  EXPECT_GT(fileSize(tmp_path), 0);
}

TEST_F(PngTest, ColormapGrayscale) {
  io::PngExportConfig cfg;
  cfg.colormap = io::PngExportConfig::Colormap::GRAYSCALE;
  ASSERT_TRUE(io::savePng(tmp_path, map, layer::elevation, cfg));
  EXPECT_GT(fileSize(tmp_path), 0);
}

TEST_F(PngTest, NormalizeFixedRange) {
  io::PngExportConfig cfg;
  cfg.normalize = io::PngExportConfig::Normalize::FIXED_RANGE;
  cfg.fixed_min = 0.0f;
  cfg.fixed_max = 5.0f;
  ASSERT_TRUE(io::savePng(tmp_path, map, layer::elevation, cfg));
  EXPECT_GT(fileSize(tmp_path), 0);
}

TEST_F(PngTest, NormalizeMinMax) {
  io::PngExportConfig cfg;
  cfg.normalize = io::PngExportConfig::Normalize::MIN_MAX;
  ASSERT_TRUE(io::savePng(tmp_path, map, layer::elevation, cfg));
  EXPECT_GT(fileSize(tmp_path), 0);
}

TEST_F(PngTest, AlignToWorldDisabled) {
  map.setStartIndex(grid_map::Index(3, 2));
  io::PngExportConfig cfg;
  cfg.align_to_world = false;
  ASSERT_TRUE(io::savePng(tmp_path, map, layer::elevation, cfg));
  EXPECT_GT(fileSize(tmp_path), 0);
}

TEST_F(PngTest, EmptyMap) {
  ElevationMap empty(4.0f, 4.0f, 0.5f, "empty");
  ASSERT_TRUE(io::savePng(tmp_path, empty, layer::elevation));
  EXPECT_GT(fileSize(tmp_path), 0);
}

TEST_F(PngTest, NonexistentLayer) {
  EXPECT_FALSE(io::savePng(tmp_path, map, "no_such_layer"));
}

TEST_F(PngTest, SaveToInvalidPath) {
  EXPECT_FALSE(io::savePng("/nonexistent/dir/test.png", map, layer::elevation));
}
