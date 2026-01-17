// nanoPCL Example 03: Channels
//
// Channels are optional - only allocate memory for what you need.
// Memory: xyz-only = 12 bytes/point, xyz+intensity = 16 bytes/point

#include <iostream>
#include <nanopcl/core.hpp>

using namespace npcl;

int main() {
  // =========================================================================
  // 1. Optional Channels - only xyz is always present
  // =========================================================================
  PointCloud cloud;

  std::cout << "=== 1. Optional Channels ===\n";
  std::cout << "hasIntensity: " << cloud.hasIntensity() << "\n";  // 0
  std::cout << "hasRing:      " << cloud.hasRing() << "\n";       // 0

  cloud.enableIntensity();
  cloud.enableRing();

  std::cout << "\nAfter enable:\n";
  std::cout << "hasIntensity: " << cloud.hasIntensity() << "\n";  // 1
  std::cout << "hasRing:      " << cloud.hasRing() << "\n";       // 1

  // =========================================================================
  // 2. Add points with channels
  // =========================================================================
  std::cout << "\n=== 2. Add Points ===\n";

  cloud.add(PointXYZIR{1, 0, 0, 0.5f, 0});
  cloud.add(PointXYZIR{2, 0, 0, 0.8f, 1});
  cloud.add(PointXYZIR{3, 0, 0, 0.3f, 2});

  std::cout << "Added 3 points with intensity and ring\n";

  // =========================================================================
  // 3. What if you don't enable first?
  // =========================================================================
  std::cout << "\n=== 3. Auto-enable Behavior ===\n";

  PointCloud cloud2;  // Fresh cloud, no channels enabled

  // Case A: DTO automatically enables required channels
  cloud2.add(PointXYZI{0, 0, 0, 1.0f});
  std::cout << "After add(PointXYZI): hasIntensity=" << cloud2.hasIntensity()
            << "\n";  // 1 (auto-enabled)

  // Case B: Variadic also auto-enables
  cloud2.add(Point(1, 0, 0), Ring(5));
  std::cout << "After add(Point, Ring): hasRing=" << cloud2.hasRing()
            << "\n";  // 1 (auto-enabled)

  // Case C: Point-only doesn't enable anything
  PointCloud cloud3;
  cloud3.add(Point(0, 0, 0));
  std::cout << "After add(Point): hasIntensity=" << cloud3.hasIntensity()
            << "\n";  // 0 (not enabled)

  // =========================================================================
  // 4. Direct Channel Access - fast, for single-channel operations
  // =========================================================================
  std::cout << "\n=== 4. Direct Channel Access ===\n";

  auto& intensities = cloud.intensity();  // Reference to internal
  for (float& i : intensities) {
    i *= 2.0f;  // Modify in-place
  }

  std::cout << "Doubled intensities: ";
  for (float i : intensities) std::cout << i << " ";  // 1.0 1.6 0.6
  std::cout << "\n";

  // =========================================================================
  // 5. PointRef Access - convenient, for multi-channel logic
  // =========================================================================
  std::cout << "\n=== 5. PointRef Access ===\n";

  // 5a. Index-based: cloud.point(i) returns PointRef
  std::cout << "Index-based (cloud.point(i)):\n";
  for (size_t i = 0; i < cloud.size(); ++i) {
    auto pt = cloud.point(i);  // Proxy to all channels at index i
    if (pt.intensity() > 1.0f && pt.ring() < 2) {
      std::cout << "  Point " << i << " matches: "
                << "intensity=" << pt.intensity() << ", ring=" << pt.ring()
                << "\n";
    }
  }

  // 5b. Range-based: iterator also returns PointRef
  //     Note: 'pt' is PointRef (proxy), not a copy of the point
  std::cout << "\nRange-based (for auto pt : cloud):\n";
  for (auto pt : cloud) {
    std::cout << "  x=" << pt.x() << ", intensity=" << pt.intensity()
              << ", ring=" << pt.ring() << "\n";
  }

  // =========================================================================
  // 6. Disable channel to free memory
  // =========================================================================
  std::cout << "\n=== 6. Disable Channel ===\n";

  std::cout << "Before: hasRing=" << cloud.hasRing() << "\n";  // 1
  cloud.disableRing();
  std::cout << "After:  hasRing=" << cloud.hasRing() << "\n";  // 0

  return 0;
}
