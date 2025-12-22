// nanoPCL Filter Example
#include <nanopcl/nanopcl.hpp>
#include <iostream>
#include <random>

int main() {
  using namespace nanopcl;

  // Generate random point cloud
  PointCloud cloud("sensor");

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> dist(-10.0f, 10.0f);
  std::uniform_real_distribution<float> z_dist(-2.0f, 5.0f);

  for (int i = 0; i < 10000; ++i) {
    cloud.push_back(Point(dist(gen), dist(gen), z_dist(gen)));
  }

  std::cout << "Original: " << cloud.size() << " points\n\n";

  // 1. Passthrough filter - keep points within bounds
  filters::Passthrough pass({
    .x_min = -5.0f, .x_max = 5.0f,
    .y_min = -5.0f, .y_max = 5.0f,
    .z_min = 0.0f, .z_max = 3.0f
  });

  auto pass_stats = pass.filterInPlace(cloud);
  std::cout << "After Passthrough:\n";
  std::cout << "  Kept: " << pass_stats.output_size << "\n";
  std::cout << "  Removed: " << pass_stats.removed_count << "\n\n";

  // 2. Voxel filter - downsample
  filters::VoxelGrid voxel({.voxel_size = 0.2f});

  auto voxel_stats = voxel.filterInPlace(cloud);
  std::cout << "After VoxelGrid:\n";
  std::cout << "  Before: " << voxel_stats.input_size << "\n";
  std::cout << "  After: " << voxel_stats.output_size << "\n";
  std::cout << "  Compression: "
            << (1.0f - float(voxel_stats.output_size) / voxel_stats.input_size) * 100
            << "%\n";

  return 0;
}
