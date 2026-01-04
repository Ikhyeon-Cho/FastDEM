// nanoPCL Example: Attribute Channels
// Demonstrates optional per-point attributes: intensity, time, ring, color,
// label

#include <iostream>
#include <nanopcl/nanopcl.hpp>
#include <stdexcept>

using namespace nanopcl;

void section(const char* title) { std::cout << "\n=== " << title << " ===\n"; }

int main() {
  std::cout << "nanoPCL " << version() << " - Attribute Channels Example\n";

  // ============================================================================
  // 1. CHANNEL LIFECYCLE
  // ============================================================================
  section("1. Channel Lifecycle");

  PointCloud cloud("lidar");

  // Check if channel is enabled (before enabling)
  std::cout << "hasIntensity(): " << (cloud.hasIntensity() ? "true" : "false")
            << "\n";

  // Enable channel
  cloud.enableIntensity();
  std::cout << "After enableIntensity(): "
            << (cloud.hasIntensity() ? "true" : "false") << "\n";

  // Disable channel (frees memory)
  cloud.disableIntensity();
  std::cout << "After disableIntensity(): "
            << (cloud.hasIntensity() ? "true" : "false") << "\n";

  // Access without enabling throws exception
  try {
    auto& intensities = cloud.intensity();
    (void)intensities;  // Suppress unused warning
  } catch (const std::runtime_error& e) {
    std::cout << "Expected error: " << e.what() << "\n";
  }

  // ============================================================================
  // 2. INTENSITY CHANNEL
  // ============================================================================
  section("2. Intensity Channel");

  cloud.enableIntensity();

  // Add points with intensity
  cloud.push_back(Point(1, 0, 0));
  cloud.point(0).intensity() = 0.5f;

  cloud.push_back(Point(2, 0, 0));
  cloud.point(1).intensity() = 0.8f;

  // Or use variadic push_back (auto-enables channel)
  cloud.push_back(Point(3, 0, 0), Intensity(0.95f));

  std::cout << "Intensities: ";
  for (size_t i = 0; i < cloud.size(); ++i) {
    std::cout << cloud.point(i).intensity() << " ";
  }
  std::cout << "\n";

  // ============================================================================
  // 3. TIME CHANNEL (for deskewing)
  // ============================================================================
  section("3. Time Channel");

  PointCloud timedCloud("velodyne");
  timedCloud.enableTime();

  // Time is typically normalized [0, 1] within a scan
  for (int i = 0; i < 5; ++i) {
    timedCloud.push_back(Point(float(i), 0, 0));
    timedCloud.point(i).time() = float(i) / 4.0f;  // 0.0, 0.25, 0.5, 0.75, 1.0
  }

  std::cout << "Times: ";
  for (size_t i = 0; i < timedCloud.size(); ++i) {
    std::cout << timedCloud.point(i).time() << " ";
  }
  std::cout << "\n";

  // Variadic push_back
  timedCloud.push_back(Point(5, 0, 0), Time(0.5f));
  std::cout << "Added point with Time(0.5), last time: "
            << timedCloud.time().back() << "\n";

  // ============================================================================
  // 4. RING CHANNEL (LiDAR layer index)
  // ============================================================================
  section("4. Ring Channel");

  PointCloud lidarCloud("ouster");
  lidarCloud.enableRing();

  // Simulate 16-channel LiDAR
  for (uint16_t ring = 0; ring < 16; ++ring) {
    for (int angle = 0; angle < 4; ++angle) {
      float x = std::cos(float(angle) * M_PI / 2);
      float y = std::sin(float(angle) * M_PI / 2);
      float z = float(ring) * 0.1f - 0.8f;  // Vertical spread

      lidarCloud.push_back(Point(x, y, z), Ring(ring));
    }
  }

  std::cout << "Total points: " << lidarCloud.size() << "\n";

  // Count points per ring
  std::vector<int> ringCount(16, 0);
  for (size_t i = 0; i < lidarCloud.size(); ++i) {
    ringCount[lidarCloud.point(i).ring()]++;
  }
  std::cout << "Points per ring: ";
  for (int c : ringCount) std::cout << c << " ";
  std::cout << "\n";

  // ============================================================================
  // 5. COLOR CHANNEL (RGB)
  // ============================================================================
  section("5. Color Channel");

  PointCloud colorCloud("realsense");
  colorCloud.enableColor();

  // Add colored points using predefined colors
  colorCloud.push_back(Point(0, 0, 0), Color::Red());
  colorCloud.push_back(Point(1, 0, 0), Color::Green());
  colorCloud.push_back(Point(2, 0, 0), Color::Blue());

  // From hex value (0xRRGGBB)
  colorCloud.push_back(Point(3, 0, 0), Color::fromHex(0xFF8800));  // Orange

  // From normalized [0,1] values
  colorCloud.push_back(Point(4, 0, 0),
                       Color::fromNormalized(0.5f, 0.0f, 0.5f));  // Purple

  std::cout << "Colors:\n";
  for (size_t i = 0; i < colorCloud.size(); ++i) {
    const auto& c = colorCloud.point(i).color();
    std::cout << "  Point " << i << ": RGB(" << int(c.r()) << ", " << int(c.g())
              << ", " << int(c.b()) << ") = 0x" << std::hex << c.toHex()
              << std::dec << "\n";
  }

  // Comparison
  std::cout << "Red == Red: "
            << (Color::Red() == Color::Red() ? "true" : "false") << "\n";
  std::cout << "Red == Blue: "
            << (Color::Red() == Color::Blue() ? "true" : "false") << "\n";

  // ============================================================================
  // 6. LABEL CHANNEL (Semantic Segmentation)
  // ============================================================================
  section("6. Label Channel");

  PointCloud semanticCloud("lidar");
  semanticCloud.enableLabel();

  // SemanticKITTI format: lower 16 bits = class, upper 16 bits = instance
  // Common classes: 0=unlabeled, 10=car, 11=bicycle, 13=bus, 15=motorcycle, ...

  // Create labeled points using strong types
  semanticCloud.push_back(
      Point(0, 0, 0),
      Label(SemanticClass(10), InstanceId(1)));  // car, instance 1
  semanticCloud.push_back(Point(1, 0, 0),
                          Label(SemanticClass(10), InstanceId(1)));  // same car
  semanticCloud.push_back(
      Point(5, 0, 0),
      Label(SemanticClass(10), InstanceId(2)));  // car, instance 2
  semanticCloud.push_back(
      Point(10, 0, 0),
      Label(SemanticClass(11)));  // bicycle (instance=0 default)

  std::cout << "Semantic labels:\n";
  for (size_t i = 0; i < semanticCloud.size(); ++i) {
    Label lbl = semanticCloud.point(i).label();
    std::cout << "  Point " << i << ": class=" << lbl.semanticClass()
              << ", instance=" << lbl.instanceId() << "\n";
  }

  // Filter by class
  int carCount = 0;
  for (size_t i = 0; i < semanticCloud.size(); ++i) {
    if (semanticCloud.point(i).label().semanticClass() == 10) {
      carCount++;
    }
  }
  std::cout << "Points labeled as 'car': " << carCount << "\n";

  // ============================================================================
  // 7. MULTIPLE CHANNELS
  // ============================================================================
  section("7. Multiple Channels");

  PointCloud multiChannel("sensor");

  // Enable multiple channels
  multiChannel.enableIntensity();
  multiChannel.enableRing();
  multiChannel.enableTime();

  // Add points with multiple attributes
  multiChannel.push_back(Point(1, 2, 3), Intensity(0.9f), Ring(5), Time(0.0f));
  multiChannel.push_back(Point(4, 5, 6), Intensity(0.7f), Ring(10), Time(0.5f));

  std::cout << "Point 0: intensity=" << multiChannel.point(0).intensity()
            << ", ring=" << multiChannel.point(0).ring()
            << ", time=" << multiChannel.point(0).time() << "\n";

  // Channels sync with container operations
  multiChannel.resize(1);
  std::cout << "After resize(1): all channels have "
            << multiChannel.intensity().size() << " elements\n";

  // ============================================================================
  // 8. CHANNEL BEHAVIOR WITH MERGE
  // ============================================================================
  section("8. Channel Behavior with Merge");

  PointCloud withIntensity("sensor");
  withIntensity.enableIntensity();
  withIntensity.push_back(Point(0, 0, 0), Intensity(1.0f));
  withIntensity.push_back(Point(1, 0, 0), Intensity(0.5f));

  PointCloud withoutIntensity("sensor");
  withoutIntensity.push_back(Point(2, 0, 0));
  withoutIntensity.push_back(Point(3, 0, 0));

  // Merge: cloud with intensity += cloud without intensity
  withIntensity += withoutIntensity;

  std::cout << "Merged intensities: ";
  for (size_t i = 0; i < withIntensity.size(); ++i) {
    std::cout << withIntensity.point(i).intensity() << " ";
  }
  std::cout << "\n";
  std::cout << "(Missing values filled with 0.0)\n";

  // ============================================================================
  // 9. CHANNEL BEHAVIOR WITH COPY
  // ============================================================================
  section("9. Channel Behavior with Copy");

  PointCloud original("original");
  original.enableIntensity();
  original.enableColor();
  original.push_back(Point(0, 0, 0), Intensity(0.8f), Color(255, 128, 64));

  // Deep copy includes all enabled channels
  PointCloud copy = original;

  std::cout << "Original: intensity=" << original.point(0).intensity()
            << ", color=(" << int(original.point(0).color().r()) << ","
            << int(original.point(0).color().g()) << ","
            << int(original.point(0).color().b()) << ")\n";

  // Modify copy (verify independence)
  copy.point(0).intensity() = 0.1f;
  copy.point(0).color() = Color(0, 0, 0);

  std::cout << "After modifying copy:\n";
  std::cout << "  Original: intensity=" << original.point(0).intensity()
            << "\n";
  std::cout << "  Copy: intensity=" << copy.point(0).intensity() << "\n";

  // ============================================================================
  // 10. POINT-CENTRIC ACCESS (PointRef)
  // ============================================================================
  section("10. Point-Centric Access (PointRef)");

  PointCloud refCloud("lidar");
  refCloud.enableIntensity();
  refCloud.enableRing();

  for (int i = 0; i < 5; ++i) {
    refCloud.push_back(Point(float(i), 0, 0), Intensity(float(i) * 0.2f), Ring(i));
  }

  // Access point and attributes together via point()
  auto p = refCloud.point(2);
  std::cout << "point(2): x=" << p.x() << ", intensity=" << p.intensity()
            << ", ring=" << p.ring() << "\n";

  // Modify via PointRef
  p.x() = 100.0f;
  p.intensity() = 0.99f;
  std::cout << "After modification: x=" << refCloud[2].x()
            << ", intensity=" << refCloud.point(2).intensity() << "\n";

  // Check channel availability before access
  if (p.hasIntensity() && !p.hasColor()) {
    std::cout << "Has intensity, no color\n";
  }

  // Const access via const overload
  const PointCloud& constRef = refCloud;
  auto cp = constRef.point(0);  // Returns ConstPointRef
  std::cout << "const point(0): x=" << cp.x()
            << ", intensity=" << cp.intensity() << "\n";

  std::cout << "\nAll attribute examples completed!\n";
  return 0;
}
