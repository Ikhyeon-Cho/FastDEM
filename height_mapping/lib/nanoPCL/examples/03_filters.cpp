// nanoPCL Example: Filters
// Demonstrates spatial filters and generic filter API with modern move semantics

#include <iostream>
#include <nanopcl/nanopcl.hpp>
#include <random>

using namespace nanopcl;

void section(const char* title) { std::cout << "\n=== " << title << " ===\n"; }

// Generate random point cloud with intensity
PointCloud generateCloud(size_t count, float range = 10.0f) {
  PointCloud cloud("sensor");
  cloud.enableIntensity();

  std::mt19937 gen(42);  // Fixed seed for reproducibility
  std::uniform_real_distribution<float> pos(-range, range);
  std::uniform_real_distribution<float> intensity(0.0f, 1.0f);

  for (size_t i = 0; i < count; ++i) {
    cloud.push_back(Point(pos(gen), pos(gen), pos(gen)),
                    Intensity(intensity(gen)));
  }
  return cloud;
}

int main() {
  std::cout << "nanoPCL " << version() << " - Filters Example\n";

  // ============================================================================
  // 1. CROP BOX - Basic Usage (Move version for optimal performance)
  // ============================================================================
  section("1. cropBox - Move Version (Optimal)");

  PointCloud cloud = generateCloud(10000);
  size_t original_size = cloud.size();
  std::cout << "Original: " << original_size << " points\n";

  // Keep points inside box - move version
  cloud = filters::cropBox(std::move(cloud), Point(-5.0f, -5.0f, -2.0f),
                           Point(5.0f, 5.0f, 2.0f));

  std::cout << "After cropBox (|x|<5, |y|<5, |z|<2):\n";
  std::cout << "  Remaining: " << cloud.size() << "\n";
  std::cout << "  Removed:   " << (original_size - cloud.size()) << "\n";

  // ============================================================================
  // 2. CROP BOX - Copy Version (Preserve Original)
  // ============================================================================
  section("2. cropBox - Copy Version (Preserve Original)");

  cloud = generateCloud(10000);
  std::cout << "Original: " << cloud.size() << " points\n";

  // Copy version - original preserved
  PointCloud roi = filters::cropBox(cloud, Point(-3.0f, -3.0f, -3.0f),
                                    Point(3.0f, 3.0f, 3.0f));

  std::cout << "Original still has: " << cloud.size() << " points\n";
  std::cout << "ROI extracted: " << roi.size() << " points\n";

  // ============================================================================
  // 3. CROP BOX - Outside Mode
  // ============================================================================
  section("3. cropBox - Outside Mode (Remove Center)");

  cloud = generateCloud(10000);
  original_size = cloud.size();
  std::cout << "Original: " << original_size << " points\n";

  // Keep points OUTSIDE box (remove center)
  cloud = filters::cropBox(std::move(cloud), Point(-3.0f, -3.0f, -3.0f),
                           Point(3.0f, 3.0f, 3.0f), filters::FilterMode::OUTSIDE);

  std::cout << "After cropBox OUTSIDE (hollow cube):\n";
  std::cout << "  Remaining: " << cloud.size() << "\n";

  // ============================================================================
  // 4. CROP RANGE - Ring Filter
  // ============================================================================
  section("4. cropRange - Ring Filter");

  cloud = generateCloud(10000);
  original_size = cloud.size();
  std::cout << "Original: " << original_size << " points\n";

  // Keep points in range [3, 8] from origin
  cloud = filters::cropRange(std::move(cloud), 3.0f, 8.0f);

  std::cout << "After cropRange (3 < range < 8):\n";
  std::cout << "  Remaining: " << cloud.size() << "\n";

  // Sample point distance check
  if (!cloud.empty()) {
    float d = cloud[0].norm();
    std::cout << "  Sample range: " << d << " (should be in [3, 8])\n";
  }

  // ============================================================================
  // 5. GENERIC FILTER - Custom Predicate
  // ============================================================================
  section("5. Generic filter - Custom Predicate");

  cloud = generateCloud(10000);
  original_size = cloud.size();
  std::cout << "Original: " << original_size << " points\n";

  // Keep points where z > 0 AND intensity > 0.5
  cloud = filters::filter(std::move(cloud), [](auto p) {
    return p.point().z() > 0 && p.hasIntensity() && p.intensity() > 0.5f;
  });

  std::cout << "After filter (z > 0 AND intensity > 0.5):\n";
  std::cout << "  Remaining: " << cloud.size() << "\n";

  // ============================================================================
  // 6. REMOVE INVALID
  // ============================================================================
  section("6. removeInvalid");

  cloud = generateCloud(1000);
  // Inject some NaN/Inf values
  cloud[0] = Point(std::numeric_limits<float>::quiet_NaN(), 0, 0);
  cloud[1] = Point(0, std::numeric_limits<float>::infinity(), 0);
  cloud[2] = Point(0, 0, -std::numeric_limits<float>::infinity());
  original_size = cloud.size();
  std::cout << "Cloud with 3 NaN/Inf points: " << original_size << " points\n";

  cloud = filters::removeInvalid(std::move(cloud));

  std::cout << "After removeInvalid:\n";
  std::cout << "  Remaining: " << cloud.size() << "\n";
  std::cout << "  Removed:   " << (original_size - cloud.size()) << " (expected: 3)\n";

  // ============================================================================
  // 7. FILTER PIPELINE - Chained Operations
  // ============================================================================
  section("7. Filter Pipeline");

  cloud = generateCloud(50000, 20.0f);  // Larger cloud
  std::cout << "Original: " << cloud.size() << " points\n";

  // Modern C++ style: chained move operations
  cloud = filters::cropRange(std::move(cloud), 0.0f, 15.0f);
  std::cout << "After range filter (range < 15): " << cloud.size() << " points\n";

  cloud = filters::cropBox(std::move(cloud), Point(-10.0f, -10.0f, -5.0f),
                           Point(10.0f, 10.0f, 5.0f));
  std::cout << "After ROI crop: " << cloud.size() << " points\n";

  cloud = filters::voxelGrid(std::move(cloud), 0.5f);
  std::cout << "After voxel grid: " << cloud.size() << " points\n";

  std::cout << "\nAll filter examples completed!\n";
  return 0;
}
