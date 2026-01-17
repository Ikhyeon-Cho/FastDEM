// nanoPCL Recipe: Performance Patterns
//
// Best practices for high-performance point cloud processing.
// SoA layout enables cache-friendly and SIMD-optimized operations.

#include <chrono>
#include <iostream>
#include <nanopcl/core.hpp>

using namespace npcl;

// Simple timer
struct Timer {
  std::chrono::high_resolution_clock::time_point start =
      std::chrono::high_resolution_clock::now();
  double us() const {
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::micro>(end - start).count();
  }
};

int main() {
  const size_t N = 100000;

  // ===========================================================================
  // 1. Direct Channel Access vs PointRef
  // ===========================================================================
  std::cout << "=== 1. Direct Channel vs PointRef ===\n";
  std::cout
      << "Rule: Single channel -> direct, Multiple channels -> PointRef\n\n";

  PointCloud cloud("sensor");
  cloud.enableIntensity();
  cloud.reserve(N);
  for (size_t i = 0; i < N; ++i) {
    cloud.add(PointXYZI(i * 0.01f, 0, 0, i * 0.00001f));
  }

  // Direct channel access (single channel operation)
  {
    Timer t;
    std::vector<float>& intensities = cloud.intensity();
    float max_i = 0;
    for (float i : intensities) max_i = std::max(max_i, i);
    for (float& i : intensities) i /= max_i;
    double time_us = t.us();
    std::cout << "[Direct] Normalize intensity: " << time_us << " us\n";
  }

  // PointRef access (multi-channel logic)
  {
    Timer t;
    std::vector<size_t> indices;
    for (size_t i = 0; i < cloud.size(); ++i) {
      auto pt = cloud.point(i);
      if (pt.x() > 500.0f && pt.intensity() > 0.5f) {
        indices.push_back(i);
      }
    }
    double time_us = t.us();
    size_t found = indices.size();
    std::cout << "[PointRef] Filter by x & intensity: " << time_us << " us, "
              << found << " points\n\n";
  }

  // ===========================================================================
  // 2. Filtering: cloud[indices] vs erase()
  // ===========================================================================
  std::cout << "=== 2. Filtering Methods ===\n";
  std::cout << "Rule: NEVER use erase() in loop. Use cloud[indices].\n\n";

  auto makeCloud = [N]() {
    PointCloud c("test");
    c.enableIntensity();
    c.reserve(N);
    for (size_t i = 0; i < N; ++i) {
      c.add(PointXYZI(i * 0.1f, 0, (i % 100) * 0.1f, 0));
    }
    return c;
  };

  // BAD: erase() in loop - O(N^2)
  {
    PointCloud c = makeCloud();
    Timer t;
    for (auto it = c.begin(); it != c.end();) {
      if (it->z() < 5.0f) {
        it = c.erase(it);
      } else {
        ++it;
      }
    }
    double time_us = t.us();
    size_t remaining = c.size();
    std::cout << "[BAD]  erase() loop: " << time_us << " us, " << remaining
              << " remaining\n";
  }

  // GOOD: cloud[indices] - O(N)
  {
    PointCloud c = makeCloud();
    Timer t;
    std::vector<size_t> keep;
    keep.reserve(c.size());
    for (size_t i = 0; i < c.size(); ++i) {
      if (c[i].z() >= 5.0f) keep.push_back(i);
    }
    PointCloud filtered = c[keep];
    double time_us = t.us();
    size_t remaining = filtered.size();
    std::cout << "[GOOD] cloud[indices]: " << time_us << " us, " << remaining
              << " remaining\n\n";
  }

  // ===========================================================================
  // 3. Bulk Insert: add() vs resize()+index
  // ===========================================================================
  std::cout << "=== 3. Bulk Insert ===\n";
  std::cout << "Rule: For large data, use resize() + direct index.\n\n";

  // add() with DTO
  {
    Timer t;
    PointCloud c;
    c.enableIntensity();
    c.reserve(N);
    for (size_t i = 0; i < N; ++i) {
      c.add(PointXYZI(i * 0.1f, i * 0.2f, i * 0.3f, i * 0.001f));
    }
    double time_us = t.us();
    std::cout << "[add()]         " << time_us << " us\n";
  }

  // resize() + direct index
  {
    Timer t;
    PointCloud c;
    c.enableIntensity();
    c.resize(N);
    auto& xyz = c.xyz();
    auto& intensity = c.intensity();
    for (size_t i = 0; i < N; ++i) {
      xyz[i] = Point(i * 0.1f, i * 0.2f, i * 0.3f);
      intensity[i] = i * 0.001f;
    }
    double time_us = t.us();
    std::cout << "[resize()+idx]  " << time_us << " us\n\n";
  }

  // ===========================================================================
  // 4. Memory: reserve() before bulk add
  // ===========================================================================
  std::cout << "=== 4. Memory Management ===\n";
  std::cout << "Rule: Always reserve() before bulk add.\n\n";

  // Without reserve
  {
    Timer t;
    PointCloud c;
    for (size_t i = 0; i < N; ++i) {
      c.add(Point(i, 0, 0));
    }
    double time_us = t.us();
    std::cout << "[No reserve]    " << time_us << " us\n";
  }

  // With reserve
  {
    Timer t;
    PointCloud c;
    c.reserve(N);
    for (size_t i = 0; i < N; ++i) {
      c.add(Point(i, 0, 0));
    }
    double time_us = t.us();
    std::cout << "[With reserve]  " << time_us << " us\n\n";
  }

  // ===========================================================================
  // 5. Summary
  // ===========================================================================
  std::cout << "=== Summary ===\n\n";
  std::cout << "| Task                  | Fast Method              |\n";
  std::cout << "|-----------------------|--------------------------|\n";
  std::cout << "| Single channel op     | xyz(), intensity()       |\n";
  std::cout << "| Multi-channel logic   | point(i), for(auto pt:)  |\n";
  std::cout << "| Filter points         | cloud[indices]           |\n";
  std::cout << "| Bulk insert           | resize() + index         |\n";
  std::cout << "| Any bulk operation    | reserve() first          |\n";

  return 0;
}
