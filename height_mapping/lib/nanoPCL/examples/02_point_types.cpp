// nanoPCL Example 02: Point Types
//
// PCL-style composite point types for convenient insertion.

#include <iostream>
#include <nanopcl/core.hpp>

using namespace npcl;

int main() {
  PointCloud cloud("sensor");

  // =========================================================================
  // Available Point Types
  // =========================================================================

  // LiDAR types
  cloud.add(PointXYZI{1, 2, 3, 0.5f});               // intensity
  cloud.add(PointXYZIR{1, 2, 3, 0.5f, 5});           // intensity + ring
  cloud.add(PointXYZIT{1, 2, 3, 0.5f, 0.001f});      // intensity + time
  cloud.add(PointXYZIRT{1, 2, 3, 0.5f, 5, 0.001f});  // intensity + ring + time

  // RGB types
  cloud.add(PointXYZRGB{1, 2, 3, 255, 0, 0});            // color
  cloud.add(PointXYZRGBN{1, 2, 3, 0, 255, 0, 0, 0, 1});  // color + normal
  cloud.add(PointXYZRGBL{1, 2, 3, 0, 0, 255, 42});       // color + label

  // Segmentation types
  cloud.add(PointXYZL{1, 2, 3, 10});         // label only
  cloud.add(PointXYZIL{1, 2, 3, 0.5f, 10});  // intensity + label

  // Normal types
  cloud.add(PointXYZN{1, 2, 3, 0, 0, 1});         // normal only
  cloud.add(PointXYZIN{1, 2, 3, 0.5f, 0, 0, 1});  // intensity + normal

  std::cout << "Added " << cloud.size()
            << " points using various point types\n\n";

  // =========================================================================
  // Channels are auto-enabled (But recommend to enable explicitly for
  // readability)
  // =========================================================================
  std::cout << "Auto-enabled channels:\n";
  std::cout << "  hasIntensity: " << cloud.hasIntensity() << "\n";
  std::cout << "  hasRing:      " << cloud.hasRing() << "\n";
  std::cout << "  hasTime:      " << cloud.hasTime() << "\n";
  std::cout << "  hasColor:     " << cloud.hasColor() << "\n";
  std::cout << "  hasLabel:     " << cloud.hasLabel() << "\n";
  std::cout << "  hasNormal:    " << cloud.hasNormal() << "\n";

  // =========================================================================
  // Alternative: Variadic API (flexible ordering)
  // =========================================================================
  std::cout << "\nVariadic API (same result, flexible order):\n";
  cloud.add(Point(1, 2, 3), Intensity(0.5f), Ring(5));
  cloud.add(Point(1, 2, 3), Ring(5), Intensity(0.5f));  // order doesn't matter
  std::cout << "  Added 2 more points\n";

  return 0;
}
