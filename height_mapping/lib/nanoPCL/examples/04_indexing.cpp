// nanoPCL Example 04: Indexing & Operations
//
// Point cloud manipulation: indexing, slicing, merging, erasing.

#include <iostream>
#include <nanopcl/core.hpp>

using namespace npcl;

int main() {
  // =========================================================================
  // 1. Setup: Create a point cloud with some data
  // =========================================================================
  PointCloud cloud("sensor");

  for (int i = 0; i < 10; ++i) {
    cloud.add(PointXYZI{float(i), float(i * 2), float(i * 3), float(i) * 0.1f});
  }

  std::cout << "=== 1. Initial Cloud ===\n";
  std::cout << "Size: " << cloud.size() << "\n\n";

  // =========================================================================
  // 2. Basic Indexing
  // =========================================================================
  std::cout << "=== 2. Basic Indexing ===\n";

  // Direct access (no bounds check)
  Point& p0 = cloud[0];
  std::cout << "cloud[0]: (" << p0.x() << ", " << p0.y() << ", " << p0.z()
            << ")\n";

  // Bounds-checked access
  Point& p1 = cloud.at(1);
  std::cout << "cloud.at(1): (" << p1.x() << ", " << p1.y() << ", " << p1.z()
            << ")\n";

  // Front and back
  std::cout << "front().x(): " << cloud.front().x() << "\n";
  std::cout << "back().x():  " << cloud.back().x() << "\n\n";

  // =========================================================================
  // 2.1 PointRef Access (Recommended)
  // =========================================================================
  std::cout << "=== 2.1 PointRef Access (Proxy) ===\n";
  
  // cloud[i] returns Point& (coordinates only)
  // cloud.point(i) returns PointRef (coordinates + attributes)
  auto pt_ref = cloud.point(0);
  std::cout << "point(0): x=" << pt_ref.x() 
            << ", intensity=" << pt_ref.intensity() << "\n";
            
  // Use PointRef to modify attributes
  pt_ref.intensity() = 1.0f;
  std::cout << "Modified intensity: " << cloud.intensity()[0] << "\n\n";

  // =========================================================================
  // 3. Fancy Indexing (Subset Extraction)
  // =========================================================================
  std::cout << "=== 3. Fancy Indexing ===\n";

  // Extract points at specific indices
  std::vector<size_t> indices = {0, 2, 5, 9};
  PointCloud subset = cloud[indices];

  std::cout << "Extracted indices {0, 2, 5, 9}:\n";
  for (size_t i = 0; i < subset.size(); ++i) {
    std::cout << "  [" << i << "] x=" << subset[i].x()
              << ", intensity=" << subset.intensity()[i] << "\n";
  }
  std::cout << "\n";

  // =========================================================================
  // 4. Container Operations
  // =========================================================================
  std::cout << "=== 4. Container Operations ===\n";

  // Note: PointCloud() defaults to capacity=65536
  // Use PointCloud(0) to start with capacity=0
  PointCloud cloud2(0);
  std::cout << "New cloud(0) - size: " << cloud2.size()
            << ", capacity: " << cloud2.capacity() << "\n";

  // Reserve memory (doesn't change size, pre-allocates)
  cloud2.reserve(1000);
  std::cout << "After reserve(1000) - size: " << cloud2.size()
            << ", capacity: " << cloud2.capacity() << "\n";

  // Add some points
  cloud2.add(PointXYZI{0, 0, 0, 0.1f});
  cloud2.add(PointXYZI{1, 0, 0, 0.2f});
  std::cout << "After adding 2 points - size: " << cloud2.size() << "\n";

  // Clear (removes all points, keeps capacity)
  cloud2.clear();
  std::cout << "After clear() - size: " << cloud2.size()
            << ", capacity: " << cloud2.capacity() << "\n\n";

  // =========================================================================
  // 5. Filtering with Fancy Indexing
  // =========================================================================
  std::cout << "=== 5. Filtering with Fancy Indexing ===\n";

  PointCloud cloud3;
  for (int i = 0; i < 10; ++i) {
    cloud3.add(PointXYZI{float(i), 0, 0, float(i) * 0.1f});
  }
  std::cout << "Original: " << cloud3.size() << " points\n";

  // Collect indices that match condition
  std::vector<size_t> keep;
  for (size_t i = 0; i < cloud3.size(); ++i) {
    if (cloud3[i].x() >= 3 && cloud3[i].x() <= 7) {  // Keep x in [3, 7]
      keep.push_back(i);
    }
  }

  // Extract subset (O(N), efficient)
  PointCloud filtered = cloud3[keep];
  std::cout << "After filtering (x in [3,7]): " << filtered.size()
            << " points\n";
  std::cout << "Filtered x values: ";
  for (size_t i = 0; i < filtered.size(); ++i) {
    std::cout << filtered[i].x() << " ";
  }
  std::cout << "\n\n";

  // =========================================================================
  // 6. Merge Operations
  // =========================================================================
  std::cout << "=== 6. Merge Operations ===\n";

  PointCloud cloudA("sensor");
  PointCloud cloudB("sensor");

  cloudA.add(PointXYZI{0, 0, 0, 0.1f});
  cloudA.add(PointXYZI{1, 0, 0, 0.2f});

  cloudB.add(PointXYZI{2, 0, 0, 0.3f});
  cloudB.add(PointXYZI{3, 0, 0, 0.4f});

  std::cout << "cloudA size: " << cloudA.size() << "\n";
  std::cout << "cloudB size: " << cloudB.size() << "\n";

  // In-place merge
  cloudA += cloudB;
  std::cout << "After cloudA += cloudB: " << cloudA.size() << " points\n";

  // Create merged copy
  PointCloud cloudC("sensor");
  cloudC.add(PointXYZI{10, 0, 0, 1.0f});

  PointCloud merged = cloudA + cloudC;
  std::cout << "cloudA + cloudC = " << merged.size() << " points\n\n";

  // =========================================================================
  // 7. Shrink to Fit
  // =========================================================================
  std::cout << "=== 7. Memory Management ===\n";

  PointCloud cloud4;
  cloud4.reserve(10000);
  cloud4.add(PointXYZI{0, 0, 0, 0});

  std::cout << "Before shrink - size: " << cloud4.size()
            << ", capacity: " << cloud4.capacity() << "\n";

  cloud4.shrink_to_fit();
  std::cout << "After shrink  - size: " << cloud4.size()
            << ", capacity: " << cloud4.capacity() << "\n";

  return 0;
}
