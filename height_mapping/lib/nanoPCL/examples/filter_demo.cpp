// Filter Demo - Save before/after PCD files for visualization
// View with: CloudCompare, PCL viewer, or Open3D
// Intensity channel encodes distance from origin for visualization

#include <iostream>
#include <nanopcl/nanopcl.hpp>
#include <random>

using namespace nanopcl;

// Generate scene with ground plane + objects at various distances
PointCloud generateCloud(size_t /*count*/, float max_range = 10.0f) {
  PointCloud cloud("sensor");
  cloud.enableIntensity();

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> noise(-0.05f, 0.05f);

  // 1. Ground plane (z = 0, grid pattern)
  for (float x = -max_range; x <= max_range; x += 0.3f) {
    for (float y = -max_range; y <= max_range; y += 0.3f) {
      float z = noise(gen);  // slight noise
      float dist = std::sqrt(x * x + y * y);
      cloud.push_back(Point(x, y, z), Intensity(dist / max_range));
    }
  }

  // 2. Vertical wall at y = 5
  for (float x = -4.0f; x <= 4.0f; x += 0.2f) {
    for (float z = 0.0f; z <= 3.0f; z += 0.2f) {
      float y = 5.0f + noise(gen);
      float dist = std::sqrt(x * x + y * y + z * z);
      cloud.push_back(Point(x, y, z), Intensity(dist / max_range));
    }
  }

  // 3. Box at (3, -3, 0) size 2x2x2
  for (float x = 2.0f; x <= 4.0f; x += 0.15f) {
    for (float y = -4.0f; y <= -2.0f; y += 0.15f) {
      for (float z = 0.0f; z <= 2.0f; z += 0.15f) {
        // Only surface points
        bool surface = (x < 2.1f || x > 3.9f || y < -3.9f || y > -2.1f ||
                        z < 0.1f || z > 1.9f);
        if (surface) {
          float dist = std::sqrt(x * x + y * y + z * z);
          cloud.push_back(Point(x, y, z), Intensity(dist / max_range));
        }
      }
    }
  }

  // 4. Sphere at (-5, 0, 1.5) radius 1.5
  float cx = -5.0f, cy = 0.0f, cz = 1.5f, r = 1.5f;
  for (int i = 0; i < 2000; ++i) {
    float theta = 2.0f * M_PI * i / 500;
    float phi = M_PI * (i % 500) / 500;
    float x = cx + r * std::sin(phi) * std::cos(theta);
    float y = cy + r * std::sin(phi) * std::sin(theta);
    float z = cz + r * std::cos(phi);
    if (z >= 0) {  // above ground only
      float dist = std::sqrt(x * x + y * y + z * z);
      cloud.push_back(Point(x, y, z), Intensity(dist / max_range));
    }
  }

  return cloud;
}

// Color by height (z-value)
void colorByHeight(PointCloud& cloud, float z_min, float z_max) {
  auto& intensity = cloud.intensity();
  for (size_t i = 0; i < cloud.size(); ++i) {
    float z = cloud[i].z();
    intensity[i] = (z - z_min) / (z_max - z_min);
  }
}

// Color by distance from origin (auto-scaled to 0~1)
void colorByDistance(PointCloud& cloud) {
  if (cloud.empty()) return;
  auto& intensity = cloud.intensity();

  // Find min/max distance
  float min_d = std::numeric_limits<float>::max();
  float max_d = 0;
  for (size_t i = 0; i < cloud.size(); ++i) {
    float d = cloud[i].norm();
    min_d = std::min(min_d, d);
    max_d = std::max(max_d, d);
  }

  // Normalize to 0~1
  float range = max_d - min_d;
  if (range < 0.001f) range = 1.0f;
  for (size_t i = 0; i < cloud.size(); ++i) {
    intensity[i] = (cloud[i].norm() - min_d) / range;
  }
}

int main() {
  std::cout << "Filter Demo - Generating PCD files\n";
  std::cout << "Intensity = distance from origin (for color mapping)\n\n";

  // Generate original cloud (intensity = distance)
  PointCloud original = generateCloud(50000, 10.0f);
  io::savePCD("01_original.pcd", original);
  std::cout << "Saved: 01_original.pcd (" << original.size() << " points)\n";

  // Demo 1: cropBox - color by distance
  PointCloud cloud =
      filters::cropBox(original, Point(-5, -5, -2), Point(5, 5, 2));
  colorByDistance(cloud);
  io::savePCD("02_cropBox.pcd", cloud);
  std::cout << "Saved: 02_cropBox.pcd (" << cloud.size() << " points)\n";

  // Demo 2: cropRange (sphere, min=0) - color by distance
  cloud = filters::cropRange(original, 0.0f, 6.0f);
  colorByDistance(cloud);
  io::savePCD("03_cropRange_sphere.pcd", cloud);
  std::cout << "Saved: 03_cropRange_sphere.pcd (" << cloud.size()
            << " points)\n";

  // Demo 3: cropRange (ring, min>0) - color by distance
  cloud = filters::cropRange(original, 4.0f, 8.0f);
  colorByDistance(cloud);
  io::savePCD("04_cropRange_ring.pcd", cloud);
  std::cout << "Saved: 04_cropRange_ring.pcd (" << cloud.size() << " points)\n";

  // Demo 4: cropBox OUTSIDE (hollow cube) - color by height
  cloud = filters::cropBox(original, Point(-3, -3, -3), Point(3, 3, 3),
                           filters::FilterMode::OUTSIDE);
  colorByHeight(cloud, -10.0f, 10.0f);
  io::savePCD("05_cropBox_hollow.pcd", cloud);
  std::cout << "Saved: 05_cropBox_hollow.pcd (" << cloud.size() << " points)\n";

  // Demo 5: Upper hemisphere (z > 0) - color by height
  cloud = filters::filter(original, [](auto p) { return p.point().z() > 0; });
  colorByHeight(cloud, 0.0f, 10.0f);
  io::savePCD("06_upper_half.pcd", cloud);
  std::cout << "Saved: 06_upper_half.pcd (" << cloud.size() << " points)\n";

  // Demo 6: Filter chain (using move semantics) - color by distance
  cloud = filters::cropRange(original, 0.0f, 8.0f);
  cloud = filters::cropBox(std::move(cloud), Point(-6, -6, -3), Point(6, 6, 3));
  cloud = filters::voxelGrid(std::move(cloud), 0.3f);
  colorByDistance(cloud);
  io::savePCD("07_filter_chain.pcd", cloud);
  std::cout << "Saved: 07_filter_chain.pcd (" << cloud.size() << " points)\n";

  std::cout << "\n=== How to view ===\n";
  std::cout << "pcl_viewer -use_point_picking 01_original.pcd\n";
  std::cout << "pcl_viewer -fc 5 *.pcd   # field coloring by intensity\n";
  std::cout << "CloudCompare: Edit > Scalar fields > Gradient\n";

  return 0;
}
