// nanoPCL Example 09: Normal Estimation
//
// Surface normal estimation using PCA (Principal Component Analysis).
// Uses VoxelHash for efficient neighbor search with zero-allocation callback.

#include <chrono>
#include <cmath>
#include <iostream>
#include <nanopcl/common.hpp>
#include <nanopcl/geometry/local_surface.hpp>
#include <random>

using namespace npcl;

// Helper: Generate planar point cloud (z = 0 with noise)
PointCloud generatePlane(size_t count, float range = 5.0f,
                         float noise = 0.05f) {
  PointCloud cloud("plane");
  cloud.reserve(count);

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> xy(-range, range);
  std::normal_distribution<float> z_noise(0.0f, noise);

  for (size_t i = 0; i < count; ++i) {
    cloud.add(Point(xy(gen), xy(gen), z_noise(gen)));
  }
  return cloud;
}

// Helper: Generate spherical point cloud
PointCloud generateSphere(size_t count, float radius = 5.0f,
                          float noise = 0.05f) {
  PointCloud cloud("sphere");
  cloud.reserve(count);

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> theta(0.0f, 2 * M_PI);
  std::uniform_real_distribution<float> phi(0.0f, M_PI);
  std::normal_distribution<float> r_noise(0.0f, noise);

  for (size_t i = 0; i < count; ++i) {
    float t = theta(gen);
    float p = phi(gen);
    float r = radius + r_noise(gen);
    cloud.add(Point(r * std::sin(p) * std::cos(t),
                    r * std::sin(p) * std::sin(t), r * std::cos(p)));
  }
  return cloud;
}

int main() {
  // ===========================================================================
  // 1. Basic Usage - Plane
  // ===========================================================================
  std::cout << "=== 1. Basic Usage (Plane) ===\n";

  PointCloud plane = generatePlane(50000);
  std::cout << "Generated " << plane.size() << " points on z=0 plane\n";

  auto start = std::chrono::high_resolution_clock::now();
  geometry::estimateNormals(plane, 0.2f);  // In-place: stored in plane.normal()
  auto end = std::chrono::high_resolution_clock::now();
  double elapsed =
      std::chrono::duration<double, std::milli>(end - start).count();

  std::cout << "Normal estimation: " << elapsed << " ms\n";

  // Verify: normals should point in +Z or -Z direction
  int valid = 0;
  float dot_sum = 0.0f;
  for (const auto& n : plane.normal()) {
    if (!n.allFinite()) continue;  // NaN check
    valid++;
    dot_sum += std::abs(n.dot(Point::UnitZ()));
  }
  std::cout << "Valid normals: " << valid << "/" << plane.size() << "\n";
  std::cout << "Avg |dot(n, Z)|: " << (dot_sum / valid) << " (ideal: 1.0)\n\n";

  // ===========================================================================
  // 2. Sphere - Normals point outward
  // ===========================================================================
  std::cout << "=== 2. Sphere (radial normals) ===\n";

  PointCloud sphere = generateSphere(50000, 5.0f);
  std::cout << "Generated " << sphere.size() << " points on sphere (r=5)\n";

  start = std::chrono::high_resolution_clock::now();
  geometry::estimateNormals(sphere, 0.3f);
  end = std::chrono::high_resolution_clock::now();
  elapsed = std::chrono::duration<double, std::milli>(end - start).count();

  std::cout << "Normal estimation: " << elapsed << " ms\n";

  // Verify: normal should align with position (radial direction)
  valid = 0;
  dot_sum = 0.0f;
  for (size_t i = 0; i < sphere.size(); ++i) {
    if (!sphere.normal()[i].allFinite()) continue;
    valid++;
    Point radial = sphere[i].normalized();
    dot_sum += std::abs(sphere.normal()[i].dot(radial));
  }
  std::cout << "Valid normals: " << valid << "/" << sphere.size() << "\n";
  std::cout << "Avg |dot(n, radial)|: " << (dot_sum / valid)
            << " (ideal: 1.0)\n\n";

  // ===========================================================================
  // 3. Parameter Tuning
  // ===========================================================================
  std::cout << "=== 3. Parameter Tuning ===\n";

  PointCloud cloud = generatePlane(30000);

  // Different radius values
  float radii[] = {0.1f, 0.2f, 0.5f, 1.0f};
  for (float r : radii) {
    PointCloud test_cloud = cloud;  // Copy for each test
    start = std::chrono::high_resolution_clock::now();
    geometry::estimateNormals(test_cloud, r);
    end = std::chrono::high_resolution_clock::now();
    elapsed = std::chrono::duration<double, std::milli>(end - start).count();

    int v = 0;
    for (const auto& normal : test_cloud.normal())
      if (normal.allFinite()) v++;

    std::cout << "radius=" << r << "m: " << elapsed << " ms, valid=" << v
              << "\n";
  }
  std::cout << "\n";

  // ===========================================================================
  // 4. min_neighbors Parameter
  // ===========================================================================
  std::cout << "=== 4. min_neighbors ===\n";

  // Sparse cloud
  PointCloud sparse = generatePlane(1000, 10.0f);
  std::cout << "Sparse cloud: " << sparse.size() << " points\n";

  size_t min_neighbors[] = {3, 5, 10, 20};
  for (size_t mn : min_neighbors) {
    PointCloud test_sparse = sparse;
    geometry::estimateNormals(test_sparse, 0.5f, mn);
    int v = 0;
    for (const auto& normal : test_sparse.normal())
      if (normal.allFinite()) v++;
    std::cout << "min_neighbors=" << mn << ": valid=" << v << "\n";
  }
  std::cout << "\n";

  // ===========================================================================
  // 5. Sample Output (PointRef access)
  // ===========================================================================
  std::cout << "=== 5. Sample Output ===\n";

  for (size_t i = 0; i < 5; ++i) {
    auto pt = plane.point(i);  // PointRef: unified access
    std::cout << "Point " << i << ": " << pt.xyz().transpose()
              << " -> Normal: " << pt.normal().transpose() << "\n";
  }

  return 0;
}
