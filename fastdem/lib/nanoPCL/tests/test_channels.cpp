// nanoPCL - Test: Channels

#include <cassert>
#include <iostream>
#include <nanopcl/core/point_cloud.hpp>

using namespace nanopcl;

#define TEST(name)                      \
  std::cout << "  " << #name << "... "; \
  test_##name();                        \
  std::cout << "OK\n"

#define ASSERT_EQ(a, b) assert((a) == (b))
#define ASSERT_NEAR(a, b, eps) assert(std::abs((a) - (b)) < (eps))
#define ASSERT_TRUE(cond) assert(cond)
#define ASSERT_FALSE(cond) assert(!(cond))

// Channel Enable

void test_initial_no_channels() {
  PointCloud cloud;
  cloud.add(1, 2, 3);

  ASSERT_FALSE(cloud.hasIntensity());
  ASSERT_FALSE(cloud.hasTime());
  ASSERT_FALSE(cloud.hasRing());
  ASSERT_FALSE(cloud.hasColor());
  ASSERT_FALSE(cloud.hasLabel());
  ASSERT_FALSE(cloud.hasNormal());
  ASSERT_FALSE(cloud.hasCovariance());
}

void test_use_intensity() {
  PointCloud cloud;
  cloud.add(1, 2, 3);
  cloud.useIntensity();

  ASSERT_TRUE(cloud.hasIntensity());
  ASSERT_EQ(cloud.intensity(0), 0.0f);
}

void test_use_normal() {
  PointCloud cloud;
  cloud.add(1, 2, 3);
  cloud.useNormal();

  ASSERT_TRUE(cloud.hasNormal());
  ASSERT_EQ(cloud.normal(0).x(), 0.0f);
  ASSERT_EQ(cloud.normal(0).y(), 0.0f);
  ASSERT_EQ(cloud.normal(0).z(), 0.0f); // default (0,0,0) = not computed
}

void test_use_covariance() {
  PointCloud cloud;
  cloud.add(1, 2, 3);
  cloud.useCovariance();

  ASSERT_TRUE(cloud.hasCovariance());
  // Default is zero matrix
  ASSERT_EQ(cloud.covariance(0)(0, 0), 0.0f);
  ASSERT_EQ(cloud.covariance(0)(1, 1), 0.0f);
  ASSERT_EQ(cloud.covariance(0)(2, 2), 0.0f);

  // Write and read
  cloud.covariance(0) = Covariance::Identity();
  ASSERT_EQ(cloud.covariance(0)(0, 0), 1.0f);
  ASSERT_EQ(cloud.covariance(0)(0, 1), 0.0f);
}

// Add with Attributes

void test_add_with_intensity() {
  PointCloud cloud;
  cloud.add(1, 2, 3, Intensity(0.5f));

  ASSERT_EQ(cloud.size(), 1);
  ASSERT_TRUE(cloud.hasIntensity());
  ASSERT_EQ(cloud.intensity(0), 0.5f);
}

void test_add_with_intensity_ring() {
  PointCloud cloud;
  cloud.add(1, 2, 3, Intensity(0.5f), Ring(3));

  ASSERT_TRUE(cloud.hasIntensity());
  ASSERT_TRUE(cloud.hasRing());
  ASSERT_EQ(cloud.intensity(0), 0.5f);
  ASSERT_EQ(cloud.ring(0), 3);
}

void test_add_with_intensity_ring_time() {
  PointCloud cloud;
  cloud.add(1, 2, 3, Intensity(0.5f), Ring(3), Time(0.001f));

  ASSERT_TRUE(cloud.hasIntensity());
  ASSERT_TRUE(cloud.hasRing());
  ASSERT_TRUE(cloud.hasTime());
  ASSERT_NEAR(cloud.time(0), 0.001f, 1e-6f);
}

// Channel Sync

void test_resize_syncs_channels() {
  PointCloud cloud;
  cloud.add(1, 2, 3, Intensity(0.5f));

  cloud.resize(10);

  ASSERT_EQ(cloud.size(), 10);
  ASSERT_EQ(cloud.intensity(0), 0.5f);
  ASSERT_EQ(cloud.intensity(9), 0.0f);
}

void test_add_syncs_existing_channels() {
  PointCloud cloud;
  cloud.add(1, 2, 3, Intensity(0.5f));
  cloud.add(4, 5, 6);

  ASSERT_EQ(cloud.size(), 2);
  ASSERT_EQ(cloud.intensity(0), 0.5f);
  ASSERT_EQ(cloud.intensity(1), 0.0f);
}

void test_clear_preserves_channel_structure() {
  PointCloud cloud;
  cloud.add(1, 2, 3, Intensity(0.5f), Ring(3));

  cloud.clear();

  ASSERT_EQ(cloud.size(), 0);
  ASSERT_TRUE(cloud.hasIntensity());
  ASSERT_TRUE(cloud.hasRing());

  cloud.add(1, 2, 3);
  ASSERT_EQ(cloud.intensity(0), 0.0f);
}

// Normal Access (Safe 3D)

void test_normal_safe_access() {
  PointCloud cloud;
  cloud.add(1, 2, 3);
  cloud.useNormal();

  // Write via head<3>()
  cloud.normal(0) = Eigen::Vector3f(1, 0, 0);

  // Read
  ASSERT_EQ(cloud.normal(0).x(), 1.0f);
  ASSERT_EQ(cloud.normal(0).y(), 0.0f);
  ASSERT_EQ(cloud.normal(0).z(), 0.0f);
}

// Extract with Channels

void test_extract_preserves_channels() {
  PointCloud cloud;
  cloud.add(0, 0, 0, Intensity(0.1f));
  cloud.add(1, 1, 1, Intensity(0.2f));
  cloud.add(2, 2, 2, Intensity(0.3f));

  PointCloud subset = cloud.extract({0, 2});

  ASSERT_EQ(subset.size(), 2);
  ASSERT_TRUE(subset.hasIntensity());
  ASSERT_EQ(subset.intensity(0), 0.1f);
  ASSERT_EQ(subset.intensity(1), 0.3f);
}

void test_extract_preserves_covariance() {
  PointCloud cloud;
  cloud.add(0, 0, 0);
  cloud.add(1, 1, 1);
  cloud.add(2, 2, 2);
  cloud.useCovariance();

  cloud.covariance(0) = Covariance::Identity() * 0.1f;
  cloud.covariance(1) = Covariance::Identity() * 0.2f;
  cloud.covariance(2) = Covariance::Identity() * 0.3f;

  PointCloud subset = cloud.extract({0, 2});

  ASSERT_EQ(subset.size(), 2);
  ASSERT_TRUE(subset.hasCovariance());
  ASSERT_NEAR(subset.covariance(0)(0, 0), 0.1f, 1e-6f);
  ASSERT_NEAR(subset.covariance(1)(0, 0), 0.3f, 1e-6f);
}

// Merge with Channels

void test_merge_syncs_channels() {
  PointCloud a, b;
  a.add(1, 1, 1, Intensity(0.1f));
  b.add(2, 2, 2, Intensity(0.2f));

  a += b;

  ASSERT_EQ(a.size(), 2);
  ASSERT_EQ(a.intensity(0), 0.1f);
  ASSERT_EQ(a.intensity(1), 0.2f);
}

void test_merge_mismatched_channels() {
  PointCloud a, b;
  a.add(1, 1, 1, Intensity(0.1f));
  b.add(2, 2, 2);

  a += b;

  ASSERT_EQ(a.size(), 2);
  ASSERT_EQ(a.intensity(0), 0.1f);
  ASSERT_EQ(a.intensity(1), 0.0f);
}

// Channel Access

void test_bulk_channel_access() {
  PointCloud cloud;
  for (int i = 0; i < 100; ++i) {
    cloud.add(float(i), float(i), float(i), Intensity(i * 0.01f));
  }

  auto& intensities = cloud.intensities();
  float sum = 0;
  for (float v : intensities) {
    sum += v;
  }

  ASSERT_NEAR(sum, 49.5f, 0.01f);
}

void test_indexed_channel_access() {
  PointCloud cloud;
  cloud.add(1, 2, 3, Intensity(0.5f));

  ASSERT_EQ(cloud.intensity(0), 0.5f);

  cloud.intensity(0) = 0.9f;
  ASSERT_EQ(cloud.intensity(0), 0.9f);
}

// Channel Utilities

void test_copy_channel_layout() {
  PointCloud src;
  src.add(1, 2, 3, Intensity(0.5f), Ring(1));
  src.useNormal();

  PointCloud dst;
  dst.copyChannelLayout(src);

  ASSERT_TRUE(dst.hasIntensity());
  ASSERT_TRUE(dst.hasRing());
  ASSERT_TRUE(dst.hasNormal());
}

void test_copy_channel_data() {
  PointCloud src;
  src.add(1, 2, 3, Intensity(0.5f));

  PointCloud dst;
  dst.copyChannelLayout(src);
  dst.resize(1);
  dst.copyChannelData(0, src, 0);

  ASSERT_EQ(dst.intensity(0), 0.5f);
}

// Alignment

void test_normal_alignment() {
  PointCloud cloud;
  cloud.useNormal();
  cloud.resize(10);

  uintptr_t addr = reinterpret_cast<uintptr_t>(cloud.normals().data());
  ASSERT_TRUE(addr % 16 == 0);
}

void test_covariance_alignment() {
  PointCloud cloud;
  cloud.useCovariance();
  cloud.resize(10);

  uintptr_t addr = reinterpret_cast<uintptr_t>(cloud.covariances().data());
  ASSERT_TRUE(addr % 16 == 0);
}

// Main

int main() {
  std::cout << "=== test_channels ===\n";

  std::cout << "Enable:\n";
  TEST(initial_no_channels);
  TEST(use_intensity);
  TEST(use_normal);
  TEST(use_covariance);

  std::cout << "Add with Attributes:\n";
  TEST(add_with_intensity);
  TEST(add_with_intensity_ring);
  TEST(add_with_intensity_ring_time);

  std::cout << "Sync:\n";
  TEST(resize_syncs_channels);
  TEST(add_syncs_existing_channels);
  TEST(clear_preserves_channel_structure);

  std::cout << "Normal Access:\n";
  TEST(normal_safe_access);

  std::cout << "Extract:\n";
  TEST(extract_preserves_channels);
  TEST(extract_preserves_covariance);

  std::cout << "Merge:\n";
  TEST(merge_syncs_channels);
  TEST(merge_mismatched_channels);

  std::cout << "Access:\n";
  TEST(bulk_channel_access);
  TEST(indexed_channel_access);

  std::cout << "Utilities:\n";
  TEST(copy_channel_layout);
  TEST(copy_channel_data);

  std::cout << "Alignment:\n";
  TEST(normal_alignment);
  TEST(covariance_alignment);

  std::cout << "\nAll tests passed!\n";
  return 0;
}
