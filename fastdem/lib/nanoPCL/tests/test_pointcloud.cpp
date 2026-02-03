// nanoPCL - Test: PointCloud

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

// Constructors

void test_default_constructor() {
  PointCloud cloud;
  ASSERT_EQ(cloud.size(), 0);
  ASSERT_TRUE(cloud.empty());
}

void test_size_constructor() {
  PointCloud cloud(100);
  ASSERT_EQ(cloud.size(), 100);
  ASSERT_FALSE(cloud.empty());
}

// Add

void test_add_xyz() {
  PointCloud cloud;
  cloud.add(1.0f, 2.0f, 3.0f);

  ASSERT_EQ(cloud.size(), 1);
  ASSERT_EQ(cloud.point(0).x(), 1.0f);
  ASSERT_EQ(cloud.point(0).y(), 2.0f);
  ASSERT_EQ(cloud.point(0).z(), 3.0f);
  ASSERT_EQ(cloud[0].w(), 1.0f);
}

void test_add_multiple() {
  PointCloud cloud;
  for (int i = 0; i < 100; ++i) {
    cloud.add(float(i), float(i), float(i));
  }

  ASSERT_EQ(cloud.size(), 100);
  ASSERT_EQ(cloud.point(50).x(), 50.0f);
}

// point() vs operator[]

void test_point_safe_access() {
  PointCloud cloud;
  cloud.add(1, 2, 3);

  // Read
  Eigen::Vector3f p = cloud.point(0);
  ASSERT_EQ(p.x(), 1.0f);

  // Write
  cloud.point(0).x() = 10.0f;
  ASSERT_EQ(cloud.point(0).x(), 10.0f);
}

void test_operator_bracket_expert() {
  PointCloud cloud;
  cloud.add(1, 2, 3);

  // Full Vector4f access
  ASSERT_EQ(cloud[0].w(), 1.0f);

  // Transform
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0, 3) = 100.0f; // Translation along x-axis
  cloud[0] = T * cloud[0];

  ASSERT_EQ(cloud.point(0).x(), 101.0f);
  ASSERT_EQ(cloud[0].w(), 1.0f);
}

void test_3d_arithmetic_safe() {
  PointCloud cloud;
  cloud.add(1, 2, 3);
  cloud.add(4, 5, 6);

  Eigen::Vector3f a = cloud.point(0);
  Eigen::Vector3f b = cloud.point(1);
  Eigen::Vector3f sum = a + b;

  ASSERT_EQ(sum.x(), 5.0f);
  ASSERT_EQ(sum.y(), 7.0f);
  ASSERT_EQ(sum.z(), 9.0f);
}

// Resize / Reserve / Clear / Reset

void test_resize() {
  PointCloud cloud;
  cloud.add(1, 2, 3);
  cloud.add(4, 5, 6);

  cloud.resize(10);
  ASSERT_EQ(cloud.size(), 10);
  ASSERT_EQ(cloud.point(0).x(), 1.0f);
  ASSERT_EQ(cloud.point(1).x(), 4.0f);
}

void test_reserve() {
  PointCloud cloud;
  cloud.reserve(1000);

  ASSERT_TRUE(cloud.capacity() >= 1000);
  ASSERT_EQ(cloud.size(), 0);
}

void test_clear_preserves_channels() {
  PointCloud cloud;
  cloud.add(1, 2, 3, Intensity(0.5f));

  cloud.clear();

  ASSERT_EQ(cloud.size(), 0);
  ASSERT_TRUE(cloud.hasIntensity());
}

void test_reset_clears_channels() {
  PointCloud cloud;
  cloud.add(1, 2, 3, Intensity(0.5f));

  cloud.reset();

  ASSERT_EQ(cloud.size(), 0);
  ASSERT_FALSE(cloud.hasIntensity());
}

// Access

void test_points_vector() {
  PointCloud cloud;
  cloud.add(1, 2, 3);
  cloud.add(4, 5, 6);

  auto& pts = cloud.points();
  ASSERT_EQ(pts.size(), 2);
  ASSERT_EQ(pts[0].x(), 1.0f);
}

void test_data_alignment() {
  PointCloud cloud(10);
  uintptr_t addr = reinterpret_cast<uintptr_t>(cloud.points().data());
  ASSERT_TRUE(addr % 16 == 0);
}

// Metadata

void test_frame_id() {
  PointCloud cloud;
  cloud.setFrameId("base_link");
  ASSERT_EQ(cloud.frameId(), "base_link");
}

void test_timestamp() {
  PointCloud cloud;
  cloud.setTimestamp(1234567890ULL);
  ASSERT_EQ(cloud.timestamp(), 1234567890ULL);
}

void test_timestamp_helpers() {
  // fromSec: seconds -> nanoseconds
  uint64_t ns = fromSec(1.5);
  ASSERT_EQ(ns, 1500000000ULL);

  // toSec: nanoseconds -> seconds
  double sec = toSec(ns);
  ASSERT_NEAR(sec, 1.5, 1e-9);

  // Round-trip
  PointCloud cloud;
  cloud.setTimestamp(fromSec(2.5));
  ASSERT_NEAR(toSec(cloud.timestamp()), 2.5, 1e-9);
}

// Extract

void test_extract() {
  PointCloud cloud;
  for (int i = 0; i < 10; ++i) {
    cloud.add(float(i), float(i), float(i));
  }

  std::vector<size_t> indices = {0, 5, 9};
  PointCloud subset = cloud.extract(indices);

  ASSERT_EQ(subset.size(), 3);
  ASSERT_EQ(subset.point(0).x(), 0.0f);
  ASSERT_EQ(subset.point(1).x(), 5.0f);
  ASSERT_EQ(subset.point(2).x(), 9.0f);
}

// Erase

void test_erase() {
  PointCloud cloud;
  for (int i = 0; i < 5; ++i) {
    cloud.add(float(i), 0, 0);
  }

  cloud.erase({0, 2, 4});

  ASSERT_EQ(cloud.size(), 2);
  ASSERT_EQ(cloud.point(0).x(), 1.0f);
  ASSERT_EQ(cloud.point(1).x(), 3.0f);
}

// Merge

void test_merge() {
  PointCloud a, b;
  a.add(1, 1, 1);
  a.add(2, 2, 2);
  b.add(3, 3, 3);

  a += b;

  ASSERT_EQ(a.size(), 3);
  ASSERT_EQ(a.point(2).x(), 3.0f);
}

// Main

int main() {
  std::cout << "=== test_pointcloud ===\n";

  std::cout << "Constructors:\n";
  TEST(default_constructor);
  TEST(size_constructor);

  std::cout << "Add:\n";
  TEST(add_xyz);
  TEST(add_multiple);

  std::cout << "Access (point vs operator[]):\n";
  TEST(point_safe_access);
  TEST(operator_bracket_expert);
  TEST(3d_arithmetic_safe);

  std::cout << "Resize/Reserve/Clear/Reset:\n";
  TEST(resize);
  TEST(reserve);
  TEST(clear_preserves_channels);
  TEST(reset_clears_channels);

  std::cout << "Vector Access:\n";
  TEST(points_vector);
  TEST(data_alignment);

  std::cout << "Metadata:\n";
  TEST(frame_id);
  TEST(timestamp);
  TEST(timestamp_helpers);

  std::cout << "Extract:\n";
  TEST(extract);

  std::cout << "Erase:\n";
  TEST(erase);

  std::cout << "Merge:\n";
  TEST(merge);

  std::cout << "\nAll tests passed!\n";
  return 0;
}
