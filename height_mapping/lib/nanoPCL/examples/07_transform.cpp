// nanoPCL Example 07: Transform
//
// Frame-aware rigid body transformation (TF2 convention).
// T_parent_child: transforms points FROM child TO parent frame.

#include <iostream>
#include <nanopcl/transform.hpp>

using namespace npcl;

int main() {
  // ===========================================================================
  // 1. Creating Transforms
  // ===========================================================================
  std::cout << "=== 1. Creating Transforms ===\n";

  // From roll-pitch-yaw (sensor mount: 0.5m up, 10deg pitch)
  Transform T_base_lidar =
      Transform::fromRPY("base_link", "lidar",           // parent, child
                         0, math::deg2rad(-10.0), 0,     // roll, pitch, yaw
                         Eigen::Vector3d(0.2, 0, 0.5));  // translation

  // From 2D pose (robot at x=5, y=3, yaw=45deg)
  auto T_map_base =
      Transform::from2D("map", "base_link",              // parent, child
                        5.0, 3.0, math::deg2rad(45.0));  // x, y, yaw

  // Identity (same frame)
  auto T_id = Transform::identity("base_link");

  std::string child = T_base_lidar.childFrame();
  std::string parent = T_base_lidar.parentFrame();
  std::cout << "T_base_lidar: " << child << " -> " << parent << "\n";

  child = T_map_base.childFrame();
  parent = T_map_base.parentFrame();
  std::cout << "T_map_base:   " << child << " -> " << parent << "\n";

  bool valid = T_map_base.isValid();
  std::cout << "isValid():    " << (valid ? "true" : "false") << "\n\n";

  // ===========================================================================
  // 2. Accessing Components
  // ===========================================================================
  std::cout << "=== 2. Accessing Components ===\n";

  // Position accessors
  double T_map_base_x = T_map_base.x();
  double T_map_base_y = T_map_base.y();
  double T_map_base_z = T_map_base.z();
  std::cout << "x(), y(), z(): "     //
            << T_map_base_x << ", "  //
            << T_map_base_y << ", "  //
            << T_map_base_z << "\n";

  // Yaw accessor
  double T_map_base_yaw = T_map_base.yaw();
  std::cout << "yaw():         " << math::rad2deg(T_map_base_yaw) << " deg\n";

  // Full translation vector
  Eigen::Vector3d T_map_base_translation = T_map_base.translation();
  std::cout << "translation(): " << T_map_base_translation.transpose() << "\n";

  // Rotation matrix
  Eigen::Matrix3d T_map_base_rotation = T_map_base.rotation();
  std::cout << "rotation():\n" << T_map_base_rotation << "\n";

  // Quaternion
  Eigen::Quaterniond T_map_base_quat = T_map_base.quaternion();
  std::cout << "quaternion() [x,y,z,w]: "
            << T_map_base_quat.coeffs().transpose() << "\n";

  // RPY extraction - individual accessors
  double roll_val = T_base_lidar.roll();
  double pitch_val = T_base_lidar.pitch();
  double yaw_val = T_base_lidar.yaw();
  std::cout << "roll():        " << math::rad2deg(roll_val) << " deg\n";
  std::cout << "pitch():       " << math::rad2deg(pitch_val) << " deg\n";
  std::cout << "yaw():         " << math::rad2deg(yaw_val) << " deg\n";

  // RPY extraction - all at once
  double roll, pitch, yaw;
  T_base_lidar.getRPY(roll, pitch, yaw);
  std::cout << "getRPY():      " << math::rad2deg(roll) << ", "
            << math::rad2deg(pitch) << ", " << math::rad2deg(yaw) << " deg\n\n";

  // ===========================================================================
  // 3. Transform a Point
  // ===========================================================================
  std::cout << "=== 3. Transform a Point ===\n";

  Eigen::Vector3d p_base(1.0, 0.0, 0.0);
  Eigen::Vector3d p_map = T_map_base * p_base;

  std::cout << "p_base:            " << p_base.transpose() << "\n";
  std::cout << "T_map_base * p_base: " << p_map.transpose() << "\n\n";

  // ===========================================================================
  // 4. Composition (Chaining)
  // ===========================================================================
  std::cout << "=== 4. Composition ===\n";

  // Chain: lidar -> base_link -> map
  Transform T_map_lidar = T_map_base * T_base_lidar;

  child = T_map_lidar.childFrame();
  parent = T_map_lidar.parentFrame();
  std::cout << "T_map_base * T_base_lidar:\n";
  std::cout << "  child:  " << child << "\n";
  std::cout << "  parent: " << parent << "\n";

  // Frame mismatch throws exception
  std::cout << "\nFrame mismatch:\n";
  try {
    auto bad = T_map_base * T_map_base;  // child != parent
    (void)bad;
  } catch (const std::runtime_error& e) {
    std::cout << "  Exception: " << e.what() << "\n\n";
  }

  // ===========================================================================
  // 5. Inverse
  // ===========================================================================
  std::cout << "=== 5. Inverse ===\n";

  Transform T_base_map = T_map_base.inverse();

  std::cout << "T_map_base.inverse():\n";
  std::cout << "  child:  " << T_base_map.childFrame() << "\n";
  std::cout << "  parent: " << T_base_map.parentFrame() << "\n\n";

  // ===========================================================================
  // 6. Interpolation (SLERP)
  // ===========================================================================
  std::cout << "=== 6. Interpolation ===\n";

  auto T_start = Transform::from2D("map", "base", 0, 0, 0);
  auto T_end = Transform::from2D("map", "base", 10, 0, math::deg2rad(90.0));

  std::cout << "T_start.slerp(T_end, t):\n";
  for (double t : {0.0, 0.5, 1.0}) {
    Transform T_interp = T_start.slerp(T_end, t);
    double interp_x = T_interp.x();
    double interp_yaw = math::rad2deg(T_interp.yaw());
    std::cout << "  t=" << t << ": x=" << interp_x << ", yaw=" << interp_yaw
              << " deg\n";
  }

  // ===========================================================================
  // 7. Comparison
  // ===========================================================================
  std::cout << "\n=== 7. Comparison ===\n";

  auto T1 = Transform::from2D("map", "base", 1.0, 2.0, 0.5);
  auto T2 = Transform::from2D("map", "base", 1.0, 2.0, 0.5);
  auto T3 = Transform::from2D("map", "base", 1.0, 2.0, 0.6);

  bool approx = T1.isApprox(T2);
  std::cout << "T1.isApprox(T2):      " << (approx ? "true" : "false") << "\n";

  approx = T1.isApprox(T3);
  std::cout << "T1.isApprox(T3):      " << (approx ? "true" : "false") << "\n";

  approx = T1.isApprox(T3, 0.2);
  std::cout << "T1.isApprox(T3, 0.2): " << (approx ? "true" : "false")
            << " (with tolerance)\n";

  bool identity = T_id.isIdentity();
  std::cout << "T_id.isIdentity():    " << (identity ? "true" : "false")
            << "\n";

  return 0;
}
