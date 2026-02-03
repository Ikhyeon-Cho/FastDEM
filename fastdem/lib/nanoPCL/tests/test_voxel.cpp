// nanoPCL - Test: Voxel Utilities

#include <cassert>
#include <cmath>
#include <iostream>
#include <nanopcl/core/voxel.hpp>

using namespace nanopcl;

#define TEST(name)                      \
  std::cout << "  " << #name << "... "; \
  test_##name();                        \
  std::cout << "OK\n"

#define ASSERT_EQ(a, b) assert((a) == (b))
#define ASSERT_NEAR(a, b, eps) assert(std::abs((a) - (b)) < (eps))
#define ASSERT_TRUE(cond) assert(cond)
#define ASSERT_FALSE(cond) assert(!(cond))

// =============================================================================
// Pack/Unpack Tests
// =============================================================================

void test_pack_unpack_roundtrip() {
  float voxel_size = 0.1f;
  float inv = 1.0f / voxel_size;

  // Test various coordinates
  struct TestCase {
    float x, y, z;
  };
  TestCase cases[] = {
      {0.0f, 0.0f, 0.0f},
      {1.0f, 2.0f, 3.0f},
      {0.05f, 0.15f, 0.25f},
      {10.5f, 20.3f, 30.7f},
  };

  for (const auto& tc : cases) {
    uint64_t key = voxel::pack(tc.x, tc.y, tc.z, inv);

    int32_t ix = voxel::unpackX(key);
    int32_t iy = voxel::unpackY(key);
    int32_t iz = voxel::unpackZ(key);

    // Verify voxel indices match floor(coord / voxel_size)
    ASSERT_EQ(ix, static_cast<int32_t>(std::floor(tc.x * inv)));
    ASSERT_EQ(iy, static_cast<int32_t>(std::floor(tc.y * inv)));
    ASSERT_EQ(iz, static_cast<int32_t>(std::floor(tc.z * inv)));
  }
}

void test_pack_negative_coordinates() {
  float voxel_size = 1.0f;
  float inv = 1.0f / voxel_size;

  // Negative coordinates
  uint64_t key = voxel::pack(-5.5f, -10.3f, -0.1f, inv);

  int32_t ix = voxel::unpackX(key);
  int32_t iy = voxel::unpackY(key);
  int32_t iz = voxel::unpackZ(key);

  ASSERT_EQ(ix, -6);  // floor(-5.5) = -6
  ASSERT_EQ(iy, -11); // floor(-10.3) = -11
  ASSERT_EQ(iz, -1);  // floor(-0.1) = -1
}

void test_pack_coordinate_clamping() {
  float inv = 1.0f;

  // Coordinates beyond 21-bit range should be clamped
  float huge = 2000000.0f; // > 2^20

  uint64_t key = voxel::pack(huge, -huge, 0.0f, inv);

  int32_t ix = voxel::unpackX(key);
  int32_t iy = voxel::unpackY(key);

  // Should be clamped to COORD_MAX and COORD_MIN
  ASSERT_EQ(ix, voxel::COORD_MAX);
  ASSERT_EQ(iy, voxel::COORD_MIN);
}

void test_pack_point4() {
  float inv = 1.0f;

  Point4 p(1.5f, 2.5f, 3.5f, 1.0f);
  uint64_t key = voxel::pack(p, inv);

  ASSERT_EQ(voxel::unpackX(key), 1);
  ASSERT_EQ(voxel::unpackY(key), 2);
  ASSERT_EQ(voxel::unpackZ(key), 3);
}

void test_pack_point3() {
  float inv = 1.0f;

  Point p(1.5f, 2.5f, 3.5f);
  uint64_t key = voxel::pack(p, inv);

  ASSERT_EQ(voxel::unpackX(key), 1);
  ASSERT_EQ(voxel::unpackY(key), 2);
  ASSERT_EQ(voxel::unpackZ(key), 3);
}

void test_pack_integer_coordinates() {
  uint64_t key = voxel::pack(10, -20, 30);

  ASSERT_EQ(voxel::unpackX(key), 10);
  ASSERT_EQ(voxel::unpackY(key), -20);
  ASSERT_EQ(voxel::unpackZ(key), 30);
}

// =============================================================================
// Key Operations Tests
// =============================================================================

void test_neighbor_key_accuracy() {
  uint64_t center = voxel::pack(0, 0, 0);

  // Test 6-connected neighbors
  ASSERT_EQ(voxel::unpackX(voxel::neighbor(center, 1, 0, 0)), 1);
  ASSERT_EQ(voxel::unpackX(voxel::neighbor(center, -1, 0, 0)), -1);
  ASSERT_EQ(voxel::unpackY(voxel::neighbor(center, 0, 1, 0)), 1);
  ASSERT_EQ(voxel::unpackY(voxel::neighbor(center, 0, -1, 0)), -1);
  ASSERT_EQ(voxel::unpackZ(voxel::neighbor(center, 0, 0, 1)), 1);
  ASSERT_EQ(voxel::unpackZ(voxel::neighbor(center, 0, 0, -1)), -1);

  // Test corner neighbor
  uint64_t corner = voxel::neighbor(center, 1, 1, 1);
  ASSERT_EQ(voxel::unpackX(corner), 1);
  ASSERT_EQ(voxel::unpackY(corner), 1);
  ASSERT_EQ(voxel::unpackZ(corner), 1);
}

void test_toCenter_accuracy() {
  float voxel_size = 0.5f;

  uint64_t key = voxel::pack(2, 4, 6);
  Point4 center = voxel::toCenter(key, voxel_size);

  // Center should be (idx + 0.5) * voxel_size
  ASSERT_NEAR(center.x(), 1.25f, 1e-5f); // (2 + 0.5) * 0.5
  ASSERT_NEAR(center.y(), 2.25f, 1e-5f); // (4 + 0.5) * 0.5
  ASSERT_NEAR(center.z(), 3.25f, 1e-5f); // (6 + 0.5) * 0.5
  ASSERT_EQ(center.w(), 1.0f);
}

void test_key_uniqueness() {
  float inv = 1.0f;

  // Different voxel coordinates should produce different keys
  uint64_t k1 = voxel::pack(0.0f, 0.0f, 0.0f, inv);
  uint64_t k2 = voxel::pack(1.0f, 0.0f, 0.0f, inv);
  uint64_t k3 = voxel::pack(0.0f, 1.0f, 0.0f, inv);
  uint64_t k4 = voxel::pack(0.0f, 0.0f, 1.0f, inv);

  ASSERT_TRUE(k1 != k2);
  ASSERT_TRUE(k1 != k3);
  ASSERT_TRUE(k1 != k4);
  ASSERT_TRUE(k2 != k3);
  ASSERT_TRUE(k2 != k4);
  ASSERT_TRUE(k3 != k4);
}

void test_same_voxel_same_key() {
  float inv = 1.0f;

  // Points in the same voxel should produce the same key
  uint64_t k1 = voxel::pack(0.1f, 0.2f, 0.3f, inv);
  uint64_t k2 = voxel::pack(0.9f, 0.8f, 0.7f, inv);

  ASSERT_EQ(k1, k2);
}

// =============================================================================
// Boundary Tests
// =============================================================================

void test_pack_at_limits() {
  // Test at coordinate limits
  int32_t max_coord = voxel::COORD_MAX;
  int32_t min_coord = voxel::COORD_MIN;

  uint64_t key_max = voxel::pack(max_coord, max_coord, max_coord);
  uint64_t key_min = voxel::pack(min_coord, min_coord, min_coord);

  ASSERT_EQ(voxel::unpackX(key_max), max_coord);
  ASSERT_EQ(voxel::unpackY(key_max), max_coord);
  ASSERT_EQ(voxel::unpackZ(key_max), max_coord);

  ASSERT_EQ(voxel::unpackX(key_min), min_coord);
  ASSERT_EQ(voxel::unpackY(key_min), min_coord);
  ASSERT_EQ(voxel::unpackZ(key_min), min_coord);
}

void test_pack_with_different_resolutions() {
  Point p(5.0f, 5.0f, 5.0f);

  // Resolution 1.0: voxel (5, 5, 5)
  uint64_t k1 = voxel::pack(p, 1.0f);
  ASSERT_EQ(voxel::unpackX(k1), 5);

  // Resolution 0.5: voxel (10, 10, 10)
  uint64_t k2 = voxel::pack(p, 2.0f);
  ASSERT_EQ(voxel::unpackX(k2), 10);

  // Resolution 2.0: voxel (2, 2, 2)
  uint64_t k3 = voxel::pack(p, 0.5f);
  ASSERT_EQ(voxel::unpackX(k3), 2);
}

void test_isValid() {
  uint64_t valid_key = voxel::pack(0, 0, 0);
  ASSERT_TRUE(voxel::isValid(valid_key));
  ASSERT_FALSE(voxel::isValid(voxel::INVALID_KEY));
}

void test_indexed_point_sorting() {
  voxel::IndexedPoint p1{voxel::pack(0, 0, 0), 0};
  voxel::IndexedPoint p2{voxel::pack(1, 0, 0), 1};
  voxel::IndexedPoint p3{voxel::pack(0, 1, 0), 2};

  ASSERT_TRUE(p1 < p2);
  ASSERT_TRUE(p1 < p3);
}

// =============================================================================
// Main
// =============================================================================

int main() {
  std::cout << "=== test_voxel ===\n";

  std::cout << "Pack/Unpack:\n";
  TEST(pack_unpack_roundtrip);
  TEST(pack_negative_coordinates);
  TEST(pack_coordinate_clamping);
  TEST(pack_point4);
  TEST(pack_point3);
  TEST(pack_integer_coordinates);

  std::cout << "Key Operations:\n";
  TEST(neighbor_key_accuracy);
  TEST(toCenter_accuracy);
  TEST(key_uniqueness);
  TEST(same_voxel_same_key);

  std::cout << "Boundary:\n";
  TEST(pack_at_limits);
  TEST(pack_with_different_resolutions);
  TEST(isValid);
  TEST(indexed_point_sorting);

  std::cout << "\nAll tests passed!\n";
  return 0;
}
