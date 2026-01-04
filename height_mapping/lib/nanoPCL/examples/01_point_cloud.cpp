// nanoPCL Example: PointCloud Core API
// Demonstrates all PointCloud operations: construction, modification, access,
// and memory management

#include <cassert>
#include <iostream>
#include <nanopcl/nanopcl.hpp>

using namespace nanopcl;

// Helper to print section headers
void section(const char* title) { std::cout << "\n=== " << title << " ===\n"; }

int main() {
  std::cout << "nanoPCL " << version() << " - PointCloud API Example\n";

  // ============================================================================
  // 1. CONSTRUCTION
  // ============================================================================
  section("1. Construction");

  // Default: reserves 10,000 points
  PointCloud cloud1;
  std::cout << "Default capacity: " << cloud1.capacity() << "\n";

  // With explicit capacity
  PointCloud cloud2(50000);
  std::cout << "Custom capacity: " << cloud2.capacity() << "\n";

  // With frame_id (most common in robotics)
  PointCloud cloud("velodyne", 1000);
  std::cout << "Frame: \"" << cloud.frameId()
            << "\", capacity: " << cloud.capacity() << "\n";

  // ============================================================================
  // 2. ADDING POINTS
  // ============================================================================
  section("2. Adding Points");

  // push_back with Point object
  cloud.push_back(Point(1.0f, 2.0f, 3.0f));

  // push_back with rvalue
  cloud.push_back(Point(4.0f, 5.0f, 6.0f));

  // Add another point
  cloud.push_back(Point(7.0f, 8.0f, 9.0f));

  std::cout << "Added 3 points, size: " << cloud.size() << "\n";

  // Bulk add
  for (int i = 0; i < 100; ++i) {
    float angle = static_cast<float>(i) * 0.1f;
    cloud.push_back(Point(std::cos(angle), std::sin(angle), 0.0f));
  }
  std::cout << "After bulk add: " << cloud.size() << " points\n";

  // ============================================================================
  // 3. POINT ACCESS
  // ============================================================================
  section("3. Point Access");

  // operator[] - no bounds check (fast)
  Point& p0 = cloud[0];
  std::cout << "cloud[0]: (" << p0.x() << ", " << p0.y() << ", " << p0.z()
            << ")\n";

  // at() - with bounds check
  try {
    Point& p1 = cloud.at(1);
    std::cout << "cloud.at(1): (" << p1.x() << ", " << p1.y() << ", " << p1.z()
              << ")\n";
  } catch (const std::out_of_range& e) {
    std::cout << "Out of range: " << e.what() << "\n";
  }

  // front() / back()
  std::cout << "front(): (" << cloud.front().transpose() << ")\n";
  std::cout << "back(): (" << cloud.back().transpose() << ")\n";

  // data() - raw pointer (for C interop or Eigen operations)
  Point* raw = cloud.data();
  std::cout << "data()[2]: (" << raw[2].transpose() << ")\n";

  // xyz() - reference to coordinates vector
  const std::vector<Point>& pts = cloud.xyz();
  std::cout << "xyz().size(): " << pts.size() << "\n";

  // ============================================================================
  // 4. ITERATORS
  // ============================================================================
  section("4. Iterators");

  // Range-based for (read)
  float sum_z = 0;
  for (const auto& pt : cloud) {
    sum_z += pt.z();
  }
  std::cout << "Sum of z: " << sum_z << "\n";

  // Iterator with modification
  for (auto it = cloud.begin(); it != cloud.end(); ++it) {
    it->z() += 0.001f;  // Small offset
  }

  // const iterators
  float min_x = std::numeric_limits<float>::max();
  for (auto it = cloud.cbegin(); it != cloud.cend(); ++it) {
    if (it->x() < min_x) min_x = it->x();
  }
  std::cout << "Min x: " << min_x << "\n";

  // Reverse iterators
  std::cout << "Last 3 points (reverse): ";
  int count = 0;
  for (auto rit = cloud.rbegin(); rit != cloud.rend() && count < 3;
       ++rit, ++count) {
    std::cout << "(" << rit->x() << ") ";
  }
  std::cout << "\n";

  // ============================================================================
  // 5. CONTAINER OPERATIONS
  // ============================================================================
  section("5. Container Operations");

  std::cout << "size(): " << cloud.size() << "\n";
  std::cout << "empty(): " << (cloud.empty() ? "true" : "false") << "\n";
  std::cout << "capacity(): " << cloud.capacity() << "\n";

  // reserve() - pre-allocate memory
  cloud.reserve(10000);
  std::cout << "After reserve(10000): capacity=" << cloud.capacity() << "\n";

  // resize() - change size (truncates or extends)
  size_t old_size = cloud.size();
  cloud.resize(50);
  std::cout << "After resize(50): size=" << cloud.size() << " (was " << old_size
            << ")\n";

  // clear() - remove all points (keeps capacity)
  cloud.clear();
  std::cout << "After clear(): size=" << cloud.size()
            << ", capacity=" << cloud.capacity() << "\n";

  // shrink_to_fit() - reduce capacity to match size
  cloud.push_back(Point(1, 2, 3));
  cloud.push_back(Point(4, 5, 6));
  std::cout << "Before shrink: size=" << cloud.size()
            << ", capacity=" << cloud.capacity() << "\n";
  cloud.shrink_to_fit();
  std::cout << "After shrink_to_fit(): size=" << cloud.size()
            << ", capacity=" << cloud.capacity() << "\n";
  cloud.clear();

  // ============================================================================
  // 6. METADATA
  // ============================================================================
  section("6. Metadata");

  // Frame ID
  cloud.setFrameId("base_link");
  std::cout << "frameId(): \"" << cloud.frameId() << "\"\n";

  // Timestamp (nanoseconds, ROS convention)
  Timestamp ts = 1703836800000000000ULL;  // 2023-12-29 00:00:00 UTC
  cloud.setTimestamp(ts);
  std::cout << "timestamp(): " << cloud.timestamp() << " ns\n";
  std::cout << "toSec(): " << time::toSec(cloud.timestamp()) << " sec\n";

  // fromSec: convert seconds to nanoseconds
  double seconds = 1703836800.5;  // with fractional part
  Timestamp fromSeconds = time::fromSec(seconds);
  std::cout << "fromSec(" << seconds << "): " << fromSeconds << " ns\n";

  // diff: time difference between two timestamps
  Timestamp t1 = time::fromSec(100.0);
  Timestamp t2 = time::fromSec(100.5);
  std::cout << "diff(t2, t1): " << time::diff(t2, t1)
            << " sec (expected: 0.5)\n";

  // Current time
  cloud.setTimestamp(time::now());
  std::cout << "now(): " << cloud.timestamp() << " ns\n";

  // ============================================================================
  // 7. MERGE CLOUDS
  // ============================================================================
  section("7. Merge Clouds");

  PointCloud cloudA("sensor");
  cloudA.push_back(Point(0, 0, 0));
  cloudA.push_back(Point(1, 0, 0));

  PointCloud cloudB("sensor");
  cloudB.push_back(Point(2, 0, 0));
  cloudB.push_back(Point(3, 0, 0));

  std::cout << "Before merge: A=" << cloudA.size() << ", B=" << cloudB.size()
            << "\n";

  // operator+= (in-place, same frame)
  cloudA += cloudB;
  std::cout << "After A += B: A=" << cloudA.size() << "\n";

  // operator+ (returns new cloud, preserves originals)
  PointCloud cloudC("sensor");
  cloudC.push_back(Point(10, 0, 0));

  PointCloud merged = cloudA + cloudC;
  std::cout << "A + C: merged=" << merged.size() << ", A=" << cloudA.size()
            << ", C=" << cloudC.size() << "\n";

  // Frame propagation: empty frame adopts other's frame
  PointCloud noFrame;  // frame = ""
  noFrame.push_back(Point(99, 0, 0));
  noFrame += cloudA;
  std::cout << "Empty frame merged with 'sensor': frame=\"" << noFrame.frameId()
            << "\"\n";

  // Frame mismatch throws exception
  PointCloud differentFrame("other_sensor");
  differentFrame.push_back(Point(0, 0, 0));
  try {
    cloudA += differentFrame;  // "sensor" vs "other_sensor"
    std::cout << "ERROR: Should have thrown!\n";
  } catch (const std::runtime_error& e) {
    std::cout << "Frame mismatch caught: " << e.what() << "\n";
  }

  // ============================================================================
  // 8. ERASE OPERATIONS
  // ============================================================================
  section("8. Erase Operations");

  PointCloud eraseTest("test");
  for (int i = 0; i < 10; ++i) {
    eraseTest.push_back(Point(float(i), 0, 0));
  }
  std::cout << "Initial: ";
  for (const auto& p : eraseTest) std::cout << int(p.x()) << " ";
  std::cout << "\n";

  // pop_back()
  eraseTest.pop_back();
  std::cout << "After pop_back(): ";
  for (const auto& p : eraseTest) std::cout << int(p.x()) << " ";
  std::cout << "\n";

  // erase by index (new!)
  eraseTest.erase(2);  // Remove index 2
  std::cout << "After erase(2): ";
  for (const auto& p : eraseTest) std::cout << int(p.x()) << " ";
  std::cout << "\n";

  // erase range by index (new!)
  eraseTest.erase(1, 3);  // Remove indices [1, 3)
  std::cout << "After erase(1, 3): ";
  for (const auto& p : eraseTest) std::cout << int(p.x()) << " ";
  std::cout << "\n";

  // Iterator versions still available for STL compatibility
  eraseTest.erase(eraseTest.begin());  // Remove first
  std::cout << "After erase(begin()): ";
  for (const auto& p : eraseTest) std::cout << int(p.x()) << " ";
  std::cout << "\n";

  // ============================================================================
  // 9. COPY AND MOVE
  // ============================================================================
  section("9. Copy and Move");

  PointCloud original("original");
  original.push_back(Point(1, 2, 3));
  original.push_back(Point(4, 5, 6));

  // Copy construction
  PointCloud copied(original);
  std::cout << "Copied size: " << copied.size() << ", frame: \""
            << copied.frameId() << "\"\n";

  // Copy assignment
  PointCloud assigned;
  assigned = original;
  std::cout << "Assigned size: " << assigned.size() << "\n";

  // Modify copy (verify deep copy)
  copied[0].x() = 999.0f;
  std::cout << "After modifying copy: original[0].x = " << original[0].x()
            << ", copied[0].x = " << copied[0].x() << "\n";
  assert(original[0].x() != copied[0].x() && "Deep copy verification");

  // Move construction
  PointCloud moved(std::move(copied));
  std::cout << "Moved size: " << moved.size()
            << ", copied size: " << copied.size() << "\n";

  // Move assignment
  PointCloud moveAssigned;
  moveAssigned = std::move(assigned);
  std::cout << "Move-assigned size: " << moveAssigned.size() << "\n";

  // swap()
  PointCloud swapA("frame_A");
  swapA.push_back(Point(1, 1, 1));
  PointCloud swapB("frame_B");
  swapB.push_back(Point(2, 2, 2));
  swapB.push_back(Point(3, 3, 3));

  std::cout << "Before swap: A.size=" << swapA.size()
            << " (frame: " << swapA.frameId() << "), B.size=" << swapB.size()
            << " (frame: " << swapB.frameId() << ")\n";
  swapA.swap(swapB);
  std::cout << "After swap:  A.size=" << swapA.size()
            << " (frame: " << swapA.frameId() << "), B.size=" << swapB.size()
            << " (frame: " << swapB.frameId() << ")\n";

  std::cout << "\nAll tests passed!\n";
  return 0;
}
