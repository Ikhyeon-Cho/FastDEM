// nanoPCL - Test: Core Types

#include <cassert>
#include <cstdint>
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

// Point4 (Vector4f)

void test_point4_construction() {
  Point4 p(1.0f, 2.0f, 3.0f, 1.0f);
  ASSERT_EQ(p.x(), 1.0f);
  ASSERT_EQ(p.y(), 2.0f);
  ASSERT_EQ(p.z(), 3.0f);
  ASSERT_EQ(p.w(), 1.0f);
}

void test_point4_alignment() {
  AlignedVector<Point4> points(10);
  uintptr_t addr = reinterpret_cast<uintptr_t>(points.data());
  ASSERT_TRUE(addr % 16 == 0);
}

// Normal4 (Vector4f)

void test_normal4_construction() {
  Normal4 n(0.0f, 0.0f, 1.0f, 0.0f);
  ASSERT_EQ(n.x(), 0.0f);
  ASSERT_EQ(n.y(), 0.0f);
  ASSERT_EQ(n.z(), 1.0f);
  ASSERT_EQ(n.w(), 0.0f);
}

void test_normal4_alignment() {
  AlignedVector<Normal4> normals(10);
  uintptr_t addr = reinterpret_cast<uintptr_t>(normals.data());
  ASSERT_TRUE(addr % 16 == 0);
}

// Strong Types

void test_intensity_type() {
  Intensity i(0.5f);
  float f = i;
  ASSERT_EQ(f, 0.5f);
}

void test_time_type() {
  Time t(0.001f);
  float f = t;
  ASSERT_NEAR(f, 0.001f, 1e-6f);
}

void test_ring_type() {
  Ring r(5);
  uint16_t v = r;
  ASSERT_EQ(v, 5);
}

void test_color_type() {
  Color c(255, 128, 0);
  ASSERT_EQ(c.r, 255);
  ASSERT_EQ(c.g, 128);
  ASSERT_EQ(c.b, 0);
}

void test_label_type() {
  Label l(42);
  uint32_t v = l;
  ASSERT_EQ(v, 42);
}

// Main

int main() {
  std::cout << "=== test_types ===\n";

  std::cout << "Point4:\n";
  TEST(point4_construction);
  TEST(point4_alignment);

  std::cout << "Normal4:\n";
  TEST(normal4_construction);
  TEST(normal4_alignment);

  std::cout << "Strong Types:\n";
  TEST(intensity_type);
  TEST(time_type);
  TEST(ring_type);
  TEST(color_type);
  TEST(label_type);

  std::cout << "\nAll tests passed!\n";
  return 0;
}
