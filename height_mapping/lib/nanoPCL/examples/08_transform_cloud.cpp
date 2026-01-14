// nanoPCL Example 08: Transforming Point Clouds
//
// Apply transforms to point clouds with frame validation.
// Copy vs Move semantics for performance.

#include <cmath>
#include <iostream>
#include <nanopcl/core.hpp>
#include <nanopcl/transform.hpp>

using namespace npcl;

// Helper: Create a simple test cloud
PointCloud createTestCloud(const std::string& frame = "lidar", size_t n = 5) {
  PointCloud cloud(frame);
  for (size_t i = 0; i < n; ++i) {
    float x = static_cast<float>(i);
    cloud.add(PointXYZI{x, 0, 0, 0.5f});
  }
  return cloud;
}

int main() {
  // ===========================================================================
  // 1. Basic Transform
  // ===========================================================================
  std::cout << "=== 1. Basic Transform ===\n";

  PointCloud cloud = createTestCloud("lidar");
  auto T_base_lidar =
      Transform::fromRPY("base_link", "lidar", {0, 0, 0}, {0, 0, 1});  // 1m up

  Point point_before = cloud[0];
  PointCloud cloud_base = transformCloud(cloud, T_base_lidar);
  Point point_after = cloud_base[0];

  std::cout << "Before: frame=" << cloud.frameId()
            << ", point[0]=" << point_before.transpose() << "\n";
  std::cout << "After:  frame=" << cloud_base.frameId()
            << ", point[0]=" << point_after.transpose() << "\n\n";

  // ===========================================================================
  // 2. Frame Validation
  // ===========================================================================
  std::cout << "=== 2. Frame Validation ===\n";

  PointCloud lidar_cloud("lidar");
  lidar_cloud.add(Point(1, 0, 0));

  // Transform expects "camera", but cloud is "lidar"
  auto T_base_camera =
      Transform::fromRPY("base_link", "camera", {0, 0, 0}, {0, 0, 0});

  std::string cloud_frame = lidar_cloud.frameId();
  std::string transform_child = T_base_camera.childFrame();
  bool match = (cloud_frame == transform_child);

  std::cout << "cloud.frameId():  " << cloud_frame << "\n";
  std::cout << "T.childFrame():   " << transform_child << "\n";
  std::cout << "Match:            " << (match ? "true" : "false") << "\n";

  try {
    auto result = transformCloud(lidar_cloud, T_base_camera);
    (void)result;
  } catch (const std::runtime_error& e) {
    std::cout << "Exception:        " << e.what() << "\n\n";
  }

  // ===========================================================================
  // 3. Copy vs Move
  // ===========================================================================
  std::cout << "=== 3. Copy vs Move ===\n";

  Transform T = Transform::fromRPY("base_link", "lidar", {0, 0, 0}, {0, 0, 1});

  // Copy: original preserved
  {
    PointCloud original = createTestCloud();
    PointCloud result = transformCloud(original, T);

    size_t original_size = original.size();
    size_t result_size = result.size();

    std::cout << "[Copy] transformCloud(cloud, T)\n";
    std::cout << "  original.size(): " << original_size << " (preserved)\n";
    std::cout << "  result.size():   " << result_size << "\n\n";
  }

  // Move: memory reused (recommended for pipelines)
  {
    PointCloud cloud = createTestCloud();
    void* ptr_before = cloud.xyz().data();

    cloud = transformCloud(std::move(cloud), T);
    void* ptr_after = cloud.xyz().data();

    bool same_memory = (ptr_before == ptr_after);

    std::cout << "[Move] transformCloud(std::move(cloud), T)\n";
    std::cout << "  Same memory: " << (same_memory ? "true" : "false") << "\n";
    std::cout << "  (Use std::move for pipeline efficiency)\n\n";
  }

  // ===========================================================================
  // 4. Transform Chain (Robotics Pipeline)
  // ===========================================================================
  std::cout << "=== 4. Transform Chain ===\n";

  // lidar -> base_link -> odom -> map
  Transform T_base_lidar2 =
      Transform::fromRPY("base_link", "lidar", {0, 0, 0}, {0.3, 0, 0.5});
  Transform T_odom_base = Transform::from2D("odom", "base_link", 2.0, 1.0, 0.5);
  Transform T_map_odom = Transform::from2D("map", "odom", 0.1, 0.0, 0.0);

  // Method 1: Transform step by step
  PointCloud scan1 = createTestCloud("lidar", 1);
  scan1 = transformCloud(std::move(scan1), T_base_lidar2);
  scan1 = transformCloud(std::move(scan1), T_odom_base);
  scan1 = transformCloud(std::move(scan1), T_map_odom);

  Point result1 = scan1[0];
  std::string result1_frame = scan1.frameId();

  std::cout << "Step-by-step:\n";
  std::cout << "  Final frame:    " << result1_frame << "\n";
  std::cout << "  Final point[0]: " << result1.transpose() << "\n";

  // Method 2: Pre-compose, then apply once
  Transform T_map_lidar = T_map_odom * T_odom_base * T_base_lidar2;
  PointCloud scan2 = createTestCloud("lidar", 1);
  scan2 = transformCloud(std::move(scan2), T_map_lidar);

  Point result2 = scan2[0];

  std::cout << "\nPre-composed:\n";
  std::cout << "  T_map_lidar = T_map_odom * T_odom_base * T_base_lidar\n";
  std::cout << "  Final point[0]: " << result2.transpose() << " (same)\n\n";

  // ===========================================================================
  // 5. Normals are Rotated (not translated)
  // ===========================================================================
  std::cout << "=== 5. Normals ===\n";

  PointCloud cloud_with_normal("sensor");
  cloud_with_normal.add(Point(1, 0, 0), Normal(0, 0, 1));  // point + normal

  Point point_before_n = cloud_with_normal[0];
  Eigen::Vector3f normal_before = cloud_with_normal.normal()[0];

  // 90 deg pitch rotation
  auto T_rot = Transform::fromRPY("rotated", "sensor",
                                  {0, math::deg2rad(90.0), 0}, {0, 0, 0});
  cloud_with_normal = transformCloud(std::move(cloud_with_normal), T_rot);

  Point point_after_n = cloud_with_normal[0];
  Eigen::Vector3f normal_after = cloud_with_normal.normal()[0];

  std::cout << "Before: point=" << point_before_n.transpose()
            << ", normal=" << normal_before.transpose() << "\n";
  std::cout << "After:  point=" << point_after_n.transpose()
            << ", normal=" << normal_after.transpose() << "\n";
  std::cout << "(Normal rotated, not translated)\n";

  return 0;
}
