// nanoPCL Example 06: Filtering
//
// Point cloud filtering: cropBox, cropRange, voxelGrid, custom predicates.
// Copy vs Move semantics for pipeline efficiency.

#include <cmath>
#include <iostream>
#include <nanopcl/common.hpp>
#include <random>

using namespace npcl;

// Helper: Generate random point cloud
PointCloud generateCloud(size_t count, float range = 10.0f) {
  PointCloud cloud("sensor");
  cloud.enableIntensity();
  cloud.enableRing();
  cloud.reserve(count);

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> pos(-range, range);
  std::uniform_real_distribution<float> intensity(0.0f, 1.0f);

  for (size_t i = 0; i < count; ++i) {
    cloud.add(Point(pos(gen), pos(gen), pos(gen)), Intensity(intensity(gen)),
              Ring(i % 16));
  }
  return cloud;
}

int main() {
  // ===========================================================================
  // 1. CropBox - Axis-aligned bounding box
  // ===========================================================================
  std::cout << "=== 1. CropBox ===\n";

  PointCloud cloud = generateCloud(10000);
  size_t original_size = cloud.size();
  std::cout << "Original: " << original_size << " points\n";

  // Keep points inside box
  Point min_pt(-5, -5, -5);
  Point max_pt(5, 5, 5);
  PointCloud inside = filters::cropBox(cloud, min_pt, max_pt);
  size_t inside_count = inside.size();
  std::cout << "cropBox INSIDE:  " << inside_count << " points\n";

  // Keep points outside box (hollow)
  PointCloud outside =
      filters::cropBox(cloud, min_pt, max_pt, filters::FilterMode::OUTSIDE);
  size_t outside_count = outside.size();
  std::cout << "cropBox OUTSIDE: " << outside_count << " points\n";
  std::cout << "Sum: " << (inside_count + outside_count) << "\n\n";

  // ===========================================================================
  // 2. CropRange - Distance-based filtering
  // ===========================================================================
  std::cout << "=== 2. CropRange ===\n";

  float min_dist = 2.0f;
  float max_dist = 8.0f;
  PointCloud ranged = filters::cropRange(cloud, min_dist, max_dist);
  size_t ranged_count = ranged.size();
  std::cout << "cropRange [" << min_dist << ", " << max_dist
            << "]: " << ranged_count << " points\n";

  // Verify distance
  float sample_dist = ranged[0].norm();
  std::cout << "Sample distance: " << sample_dist << "m\n\n";

  // ===========================================================================
  // 3. VoxelGrid - Downsampling
  // ===========================================================================
  std::cout << "=== 3. VoxelGrid ===\n";

  try {
    float voxel_size = 0.5f;
    PointCloud downsampled = filters::voxelGrid(cloud, voxel_size);
    size_t down_count = downsampled.size();
    float ratio = 100.0f * down_count / original_size;
    std::cout << "voxelGrid(" << voxel_size << "m): " << down_count
              << " points (" << ratio << "%)\n";

    // Safety check example
    // filters::voxelGrid(cloud, 0.0001f); // This would throw (0.1mm < 1mm limit)
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
  }
  std::cout << "\n";

  // ===========================================================================
  // 3.1 CropAngle - FOV Filtering (New Feature)
  // ===========================================================================
  std::cout << "=== 3.1 CropAngle ===\n";
  
  // Keep front 90 degrees (+/- 45 deg)
  float fov = 45.0f * M_PI / 180.0f;
  auto front_view = filters::cropAngle(cloud, -fov, fov);
  std::cout << "Front view (+/- 45 deg): " << front_view.size() << " points\n";

  // Wrap-around: Keep rear 90 degrees (using min > max)
  auto rear_view = filters::cropAngle(cloud, M_PI - fov, -M_PI + fov);
  std::cout << "Rear view (wrap-around): " << rear_view.size() << " points\n\n";

  // ===========================================================================
  // 4. Custom Predicate - Full flexibility
  // ===========================================================================
  std::cout << "=== 4. Custom Predicate ===\n";

  // Filter: z > 0 AND intensity > 0.5 AND ring < 8
  PointCloud filtered = filters::filter(cloud, [](auto p) {
    return p.z() > 0 && p.intensity() > 0.5f && p.ring() < 8;
  });
  size_t filtered_count = filtered.size();
  std::cout << "Custom (z>0, i>0.5, ring<8): " << filtered_count
            << " points\n\n";

  // ===========================================================================
  // 5. Move vs Copy
  // ===========================================================================
  std::cout << "=== 5. Move vs Copy ===\n";

  // COPY: Original preserved
  PointCloud src = generateCloud(5000);
  PointCloud copy_result = filters::cropBox(src, min_pt, max_pt);
  size_t src_after_copy = src.size();
  std::cout << "COPY: src remains " << src_after_copy << " points\n";

  // MOVE: Original emptied (faster)
  PointCloud move_result = filters::cropBox(std::move(src), min_pt, max_pt);
  size_t src_after_move = src.size();
  std::cout << "MOVE: src becomes " << src_after_move
            << " points (emptied)\n\n";

  // ===========================================================================
  // 6. Filter Chaining (Pipeline)
  // ===========================================================================
  std::cout << "=== 6. Filter Chaining ===\n";

  PointCloud pipeline = generateCloud(50000, 20.0f);
  size_t n0 = pipeline.size();
  std::cout << "Start: " << n0 << " points\n";

  // Chain with std::move for efficiency
  pipeline = filters::cropRange(std::move(pipeline), 1.0f, 15.0f);
  size_t n1 = pipeline.size();
  std::cout << "After cropRange:  " << n1 << " points\n";

  pipeline = filters::cropBox(std::move(pipeline), Point(0, -10, -3),
                              Point(15, 10, 3));
  size_t n2 = pipeline.size();
  std::cout << "After cropBox:    " << n2 << " points\n";

  pipeline = filters::voxelGrid(std::move(pipeline), 0.2f);
  size_t n3 = pipeline.size();
  std::cout << "After voxelGrid:  " << n3 << " points\n";

  return 0;
}
