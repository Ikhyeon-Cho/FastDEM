// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Implementation of PointRefImpl methods.
// Do not include directly; included by point_cloud.hpp after PointCloud is
// complete.

#ifndef NANOPCL_CORE_IMPL_POINT_ACCESSORS_IMPL_HPP
#define NANOPCL_CORE_IMPL_POINT_ACCESSORS_IMPL_HPP

namespace npcl {

// =============================================================================
// PointRefImpl - Const accessors (available for both IsConst=true and false)
// =============================================================================

template <bool IsConst>
const Point& PointRefImpl<IsConst>::xyz() const {
  return cloud_[idx_];
}

template <bool IsConst>
float PointRefImpl<IsConst>::x() const {
  return cloud_[idx_].x();
}

template <bool IsConst>
float PointRefImpl<IsConst>::y() const {
  return cloud_[idx_].y();
}

template <bool IsConst>
float PointRefImpl<IsConst>::z() const {
  return cloud_[idx_].z();
}

// --- Channel checks ---

template <bool IsConst>
bool PointRefImpl<IsConst>::hasIntensity() const {
  return cloud_.hasIntensity();
}

template <bool IsConst>
bool PointRefImpl<IsConst>::hasTime() const {
  return cloud_.hasTime();
}

template <bool IsConst>
bool PointRefImpl<IsConst>::hasRing() const {
  return cloud_.hasRing();
}

template <bool IsConst>
bool PointRefImpl<IsConst>::hasColor() const {
  return cloud_.hasColor();
}

template <bool IsConst>
bool PointRefImpl<IsConst>::hasLabel() const {
  return cloud_.hasLabel();
}

template <bool IsConst>
bool PointRefImpl<IsConst>::hasNormal() const {
  return cloud_.hasNormal();
}

// --- Const attribute access ---

template <bool IsConst>
float PointRefImpl<IsConst>::intensity() const {
  return cloud_.intensity()[idx_];
}

template <bool IsConst>
float PointRefImpl<IsConst>::time() const {
  return cloud_.time()[idx_];
}

template <bool IsConst>
uint16_t PointRefImpl<IsConst>::ring() const {
  return cloud_.ring()[idx_];
}

template <bool IsConst>
const Color& PointRefImpl<IsConst>::color() const {
  return cloud_.color()[idx_];
}

template <bool IsConst>
const Label& PointRefImpl<IsConst>::label() const {
  return cloud_.label()[idx_];
}

template <bool IsConst>
const Eigen::Vector3f& PointRefImpl<IsConst>::normal() const {
  return cloud_.normal()[idx_];
}

// =============================================================================
// PointRefImpl - Mutable accessors (only for IsConst=false)
// =============================================================================

template <bool IsConst>
template <bool C, typename>
Point& PointRefImpl<IsConst>::xyz() {
  return cloud_[idx_];
}

template <bool IsConst>
template <bool C, typename>
float& PointRefImpl<IsConst>::x() {
  return cloud_[idx_].x();
}

template <bool IsConst>
template <bool C, typename>
float& PointRefImpl<IsConst>::y() {
  return cloud_[idx_].y();
}

template <bool IsConst>
template <bool C, typename>
float& PointRefImpl<IsConst>::z() {
  return cloud_[idx_].z();
}

template <bool IsConst>
template <bool C, typename>
float& PointRefImpl<IsConst>::intensity() {
  return cloud_.intensity()[idx_];
}

template <bool IsConst>
template <bool C, typename>
float& PointRefImpl<IsConst>::time() {
  return cloud_.time()[idx_];
}

template <bool IsConst>
template <bool C, typename>
uint16_t& PointRefImpl<IsConst>::ring() {
  return cloud_.ring()[idx_];
}

template <bool IsConst>
template <bool C, typename>
Color& PointRefImpl<IsConst>::color() {
  return cloud_.color()[idx_];
}

template <bool IsConst>
template <bool C, typename>
Label& PointRefImpl<IsConst>::label() {
  return cloud_.label()[idx_];
}

template <bool IsConst>
template <bool C, typename>
Eigen::Vector3f& PointRefImpl<IsConst>::normal() {
  return cloud_.normal()[idx_];
}

}  // namespace npcl

#endif  // NANOPCL_CORE_IMPL_POINT_ACCESSORS_IMPL_HPP
