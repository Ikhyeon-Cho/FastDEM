// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Point accessors: PointRef, Iterator, and related proxy classes.
// These provide unified access to SoA-stored point data.

#ifndef NANOPCL_CORE_POINT_ACCESSORS_HPP
#define NANOPCL_CORE_POINT_ACCESSORS_HPP

#include <cstddef>
#include <iterator>
#include <type_traits>

#include "nanopcl/core/point.hpp"

namespace npcl {

// Forward declaration
class PointCloud;

// =============================================================================
// PointRefImpl - Unified reference proxy for point access (mutable/const)
// =============================================================================

/**
 * @brief Proxy object representing a reference to a single point in the SoA layout
 *
 * This class acts like a reference to a struct (e.g., PointXYZI&), but internally
 * it holds pointers to the underlying parallel arrays (SoA).
 *
 * - Behaving like a reference: Modifications through this object affect the original PointCloud.
 * - Lightweight: Contains only a pointer to the cloud and an index.
 * - Unified: Handles both const and mutable access via template parameter.
 *
 * @tparam IsConst true for ConstPointRef (read-only), false for PointRef (read-write)
 */
template <bool IsConst>
class PointRefImpl {
 public:
  using CloudType = std::conditional_t<IsConst, const PointCloud, PointCloud>;
  using PointType = std::conditional_t<IsConst, const Point, Point>;

  PointRefImpl(CloudType& cloud, size_t idx) : cloud_(cloud), idx_(idx) {}

  // Allow conversion from mutable to const
  template <bool C = IsConst, typename = std::enable_if_t<C>>
  PointRefImpl(const PointRefImpl<false>& other)
      : cloud_(other.cloud()), idx_(other.index()) {}

  [[nodiscard]] size_t index() const { return idx_; }
  [[nodiscard]] CloudType& cloud() const { return cloud_; }

  // --- Coordinates (XYZ, X, Y, Z) ---
  
  /// Get reference to the underlying Point (Eigen::Vector3f)
  const Point& xyz() const;
  
  /// Get mutable reference to the underlying Point
  template <bool C = IsConst, typename = std::enable_if_t<!C>>
  Point& xyz();

  /// Get X coordinate
  float x() const;
  template <bool C = IsConst, typename = std::enable_if_t<!C>>
  float& x();

  /// Get Y coordinate
  float y() const;
  template <bool C = IsConst, typename = std::enable_if_t<!C>>
  float& y();

  /// Get Z coordinate
  float z() const;
  template <bool C = IsConst, typename = std::enable_if_t<!C>>
  float& z();

  // --- Intensity ---
  [[nodiscard]] bool hasIntensity() const;
  
  /// Get intensity value
  float intensity() const;
  
  /// Get mutable reference to intensity
  template <bool C = IsConst, typename = std::enable_if_t<!C>>
  float& intensity();

  // --- Time ---
  [[nodiscard]] bool hasTime() const;
  float time() const;
  template <bool C = IsConst, typename = std::enable_if_t<!C>>
  float& time();

  // --- Ring ---
  [[nodiscard]] bool hasRing() const;
  uint16_t ring() const;
  template <bool C = IsConst, typename = std::enable_if_t<!C>>
  uint16_t& ring();

  // --- Color ---
  [[nodiscard]] bool hasColor() const;
  const Color& color() const;
  template <bool C = IsConst, typename = std::enable_if_t<!C>>
  Color& color();

  // --- Label ---
  [[nodiscard]] bool hasLabel() const;
  const Label& label() const;
  template <bool C = IsConst, typename = std::enable_if_t<!C>>
  Label& label();

  // --- Normal ---
  [[nodiscard]] bool hasNormal() const;
  const Eigen::Vector3f& normal() const;
  template <bool C = IsConst, typename = std::enable_if_t<!C>>
  Eigen::Vector3f& normal();

 private:
  CloudType& cloud_;
  size_t idx_;
};

// Type aliases
using PointRef = PointRefImpl<false>;
using ConstPointRef = PointRefImpl<true>;

// =============================================================================
// Internal implementation details
// =============================================================================
namespace detail {

/// Arrow proxy for iterator's operator-> support (internal use only)
template <bool IsConst>
class ArrowProxyImpl {
 public:
  using RefType = PointRefImpl<IsConst>;
  using CloudType = typename RefType::CloudType;

  ArrowProxyImpl(CloudType& cloud, size_t idx) : ref_(cloud, idx) {}

  std::conditional_t<IsConst, const RefType*, RefType*> operator->() {
    return &ref_;
  }

 private:
  RefType ref_;
};

}  // namespace detail

// =============================================================================
// PointRefIteratorImpl - Unified random access iterator (mutable/const)
// =============================================================================
template <bool IsConst>
class PointRefIteratorImpl {
 public:
  using CloudType = std::conditional_t<IsConst, const PointCloud, PointCloud>;
  using RefType = PointRefImpl<IsConst>;

  // STL iterator traits
  using iterator_category = std::random_access_iterator_tag;
  using value_type = RefType;
  using difference_type = std::ptrdiff_t;
  using pointer = detail::ArrowProxyImpl<IsConst>;
  using reference = RefType;

  // --- Constructors ---
  PointRefIteratorImpl() : cloud_(nullptr), idx_(0) {}
  PointRefIteratorImpl(CloudType& cloud, size_t idx)
      : cloud_(&cloud), idx_(idx) {}

  // Allow conversion from mutable to const iterator
  template <bool C = IsConst, typename = std::enable_if_t<C>>
  PointRefIteratorImpl(const PointRefIteratorImpl<false>& other)
      : cloud_(other.cloud_ptr()), idx_(other.index()) {}

  // --- Access ---
  [[nodiscard]] CloudType* cloud_ptr() const { return cloud_; }
  [[nodiscard]] size_t index() const { return idx_; }

  // --- Dereference ---
  RefType operator*() const { return RefType(*cloud_, idx_); }
  pointer operator->() const {
    return detail::ArrowProxyImpl<IsConst>(*cloud_, idx_);
  }
  RefType operator[](difference_type n) const {
    return RefType(*cloud_, idx_ + n);
  }

  // --- Increment/Decrement ---
  PointRefIteratorImpl& operator++() {
    ++idx_;
    return *this;
  }
  PointRefIteratorImpl operator++(int) {
    auto tmp = *this;
    ++idx_;
    return tmp;
  }
  PointRefIteratorImpl& operator--() {
    --idx_;
    return *this;
  }
  PointRefIteratorImpl operator--(int) {
    auto tmp = *this;
    --idx_;
    return tmp;
  }

  // --- Arithmetic ---
  PointRefIteratorImpl& operator+=(difference_type n) {
    idx_ += n;
    return *this;
  }
  PointRefIteratorImpl& operator-=(difference_type n) {
    idx_ -= n;
    return *this;
  }
  PointRefIteratorImpl operator+(difference_type n) const {
    return PointRefIteratorImpl(*cloud_, idx_ + n);
  }
  PointRefIteratorImpl operator-(difference_type n) const {
    return PointRefIteratorImpl(*cloud_, idx_ - n);
  }
  difference_type operator-(const PointRefIteratorImpl& other) const {
    return static_cast<difference_type>(idx_) -
           static_cast<difference_type>(other.idx_);
  }

  // --- Comparison ---
  bool operator==(const PointRefIteratorImpl& other) const {
    return idx_ == other.idx_;
  }
  bool operator!=(const PointRefIteratorImpl& other) const {
    return idx_ != other.idx_;
  }
  bool operator<(const PointRefIteratorImpl& other) const {
    return idx_ < other.idx_;
  }
  bool operator<=(const PointRefIteratorImpl& other) const {
    return idx_ <= other.idx_;
  }
  bool operator>(const PointRefIteratorImpl& other) const {
    return idx_ > other.idx_;
  }
  bool operator>=(const PointRefIteratorImpl& other) const {
    return idx_ >= other.idx_;
  }

 private:
  // Friend declaration for const/non-const conversion
  template <bool>
  friend class PointRefIteratorImpl;

  CloudType* cloud_;
  size_t idx_;
};

// Free function for n + iterator
template <bool IsConst>
PointRefIteratorImpl<IsConst> operator+(
    typename PointRefIteratorImpl<IsConst>::difference_type n,
    const PointRefIteratorImpl<IsConst>& it) {
  return it + n;
}

// Type aliases
using PointRefIterator = PointRefIteratorImpl<false>;
using ConstPointRefIterator = PointRefIteratorImpl<true>;

}  // namespace npcl

#endif  // NANOPCL_CORE_POINT_ACCESSORS_HPP
