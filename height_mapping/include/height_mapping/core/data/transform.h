/*
 * transform.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_DATA_TRANSFORM_H
#define HEIGHT_MAPPING_CORE_DATA_TRANSFORM_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "height_mapping/core/data/point_cloud.h"

namespace height_mapping {

// 3D rigid transformation (rotation + translation)
class Transform3D {
 private:
  Eigen::Matrix4f matrix_;

 public:
  // Constructors
  Transform3D() : matrix_(Eigen::Matrix4f::Identity()) {}

  explicit Transform3D(const Eigen::Matrix4f &matrix) : matrix_(matrix) {}

  Transform3D(const Eigen::Matrix3f &rotation,
              const Eigen::Vector3f &translation) {
    matrix_ = Eigen::Matrix4f::Identity();
    matrix_.block<3, 3>(0, 0) = rotation;
    matrix_.block<3, 1>(0, 3) = translation;
  }

  Transform3D(const Eigen::Quaternionf &rotation,
              const Eigen::Vector3f &translation) {
    matrix_ = Eigen::Matrix4f::Identity();
    matrix_.block<3, 3>(0, 0) = rotation.toRotationMatrix();
    matrix_.block<3, 1>(0, 3) = translation;
  }

  // Factory methods
  static Transform3D Identity() { return Transform3D(); }
  static Transform3D fromTranslation(float x, float y, float z) {
    Transform3D tf;
    tf.matrix_(0, 3) = x;
    tf.matrix_(1, 3) = y;
    tf.matrix_(2, 3) = z;
    return tf;
  }
  static Transform3D fromTranslation(const Eigen::Vector3f &translation) {
    return fromTranslation(translation.x(), translation.y(), translation.z());
  }

  static Transform3D fromRotationRPY(float roll, float pitch, float yaw) {
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

    Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
    return Transform3D(q, Eigen::Vector3f::Zero());
  }
  static Transform3D fromQuaternion(float x, float y, float z, float w) {
    Eigen::Quaternionf q(w, x, y, z);
    return Transform3D(q, Eigen::Vector3f::Zero());
  }

  // Getters
  Eigen::Matrix4f matrix() const { return matrix_; }
  Eigen::Matrix3f rotation() const { return matrix_.block<3, 3>(0, 0); }
  Eigen::Vector3f translation() const { return matrix_.block<3, 1>(0, 3); }

  // Convenience accessors for position
  float x() const { return matrix_(0, 3); }
  float y() const { return matrix_(1, 3); }
  float z() const { return matrix_(2, 3); }

  Eigen::Quaternionf quaternion() const {
    return Eigen::Quaternionf(rotation());
  }

  void getRPY(float &roll, float &pitch, float &yaw) const {
    Eigen::Vector3f rpy = rotation().eulerAngles(0, 1, 2);
    roll = rpy[0];
    pitch = rpy[1];
    yaw = rpy[2];
  }

  // Setters
  void setTranslation(const Eigen::Vector3f &translation) {
    matrix_.block<3, 1>(0, 3) = translation;
  }
  void setRotation(const Eigen::Matrix3f &rotation) {
    matrix_.block<3, 3>(0, 0) = rotation;
  }
  void setRotation(const Eigen::Quaternionf &rotation) {
    matrix_.block<3, 3>(0, 0) = rotation.toRotationMatrix();
  }

  // Transform operations
  Point3D apply(const Point3D &point) const {
    Eigen::Vector4f p(point.x, point.y, point.z, 1.0f);
    Eigen::Vector4f transformed = matrix_ * p;
    return Point3D(transformed[0], transformed[1], transformed[2]);
  }

  Eigen::Vector3f apply(const Eigen::Vector3f &vec) const {
    Eigen::Vector4f v(vec.x(), vec.y(), vec.z(), 1.0f);
    Eigen::Vector4f transformed = matrix_ * v;
    return Eigen::Vector3f(transformed[0], transformed[1], transformed[2]);
  }

  // Transform composition
  Transform3D operator*(const Transform3D &other) const {
    return Transform3D(matrix_ * other.matrix_);
  }

  Transform3D &operator*=(const Transform3D &other) {
    matrix_ = matrix_ * other.matrix_;
    return *this;
  }

  // Inverse transform
  Transform3D inverse() const { return Transform3D(matrix_.inverse()); }

  // Interpolation (SLERP for rotation, linear for translation)
  static Transform3D interpolate(const Transform3D &t1, const Transform3D &t2,
                                 float alpha) {
    // Interpolate translation
    Eigen::Vector3f trans =
        (1 - alpha) * t1.translation() + alpha * t2.translation();

    // Interpolate rotation using SLERP
    Eigen::Quaternionf q1 = t1.quaternion();
    Eigen::Quaternionf q2 = t2.quaternion();
    Eigen::Quaternionf q_interp = q1.slerp(alpha, q2);

    return Transform3D(q_interp, trans);
  }

  // Comparison
  bool isApprox(const Transform3D &other, float tolerance = 1e-6f) const {
    return matrix_.isApprox(other.matrix_, tolerance);
  }

  bool isIdentity(float tolerance = 1e-6f) const {
    return matrix_.isApprox(Eigen::Matrix4f::Identity(), tolerance);
  }

  // String representation (for debugging)
  std::string toString() const {
    std::stringstream ss;
    ss << "Transform3D:\n";
    ss << "  Translation: [" << matrix_(0, 3) << ", " << matrix_(1, 3) << ", "
       << matrix_(2, 3) << "]\n";

    float roll, pitch, yaw;
    getRPY(roll, pitch, yaw);
    ss << "  Rotation (RPY): [" << roll << ", " << pitch << ", " << yaw
       << "]\n";

    return ss.str();
  }
};

// 2D transformation for planar operations
class Transform2D {
 private:
  float x_, y_, theta_;

 public:
  Transform2D() : x_(0), y_(0), theta_(0) {}
  Transform2D(float x, float y, float theta) : x_(x), y_(y), theta_(theta) {}

  // Getters
  float x() const { return x_; }
  float y() const { return y_; }
  float theta() const { return theta_; }

  // Apply to 2D point
  std::pair<float, float> apply(float px, float py) const {
    float cos_theta = std::cos(theta_);
    float sin_theta = std::sin(theta_);

    float tx = cos_theta * px - sin_theta * py + x_;
    float ty = sin_theta * px + cos_theta * py + y_;

    return {tx, ty};
  }

  // Convert to 3D transform (assuming z=0 plane)
  Transform3D to3D() const {
    return Transform3D::fromTranslation(x_, y_, 0) *
           Transform3D::fromRotationRPY(0, 0, theta_);
  }

  // Inverse
  Transform2D inverse() const {
    float cos_theta = std::cos(-theta_);
    float sin_theta = std::sin(-theta_);

    float inv_x = -cos_theta * x_ - sin_theta * y_;
    float inv_y = sin_theta * x_ - cos_theta * y_;

    return Transform2D(inv_x, inv_y, -theta_);
  }

  // Composition
  Transform2D operator*(const Transform2D &other) const {
    float cos_theta = std::cos(theta_);
    float sin_theta = std::sin(theta_);

    float new_x = x_ + cos_theta * other.x_ - sin_theta * other.y_;
    float new_y = y_ + sin_theta * other.x_ + cos_theta * other.y_;
    float new_theta = theta_ + other.theta_;

    return Transform2D(new_x, new_y, new_theta);
  }
};

}  // namespace height_mapping

#endif  // HEIGHT_MAPPING_CORE_DATA_TRANSFORM_H