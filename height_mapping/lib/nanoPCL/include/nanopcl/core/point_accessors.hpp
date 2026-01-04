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

namespace nanopcl {

// Forward declaration
class PointCloud;

// =============================================================================
// PointRefImpl - Unified reference proxy for point access (mutable/const)
// =============================================================================
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

  // --- Point access (const) ---
  const Point& point() const;
  float x() const;
  float y() const;
  float z() const;

  // --- Point access (mutable, only when IsConst=false) ---
  template <bool C = IsConst, typename = std::enable_if_t<!C>>
  Point& point();

  template <bool C = IsConst, typename = std::enable_if_t<!C>>
  float& x();

  template <bool C = IsConst, typename = std::enable_if_t<!C>>
  float& y();

  template <bool C = IsConst, typename = std::enable_if_t<!C>>
  float& z();

  // --- Channel availability checks ---
  [[nodiscard]] bool hasIntensity() const;
  [[nodiscard]] bool hasTime() const;
  [[nodiscard]] bool hasRing() const;
  [[nodiscard]] bool hasColor() const;
  [[nodiscard]] bool hasLabel() const;
  [[nodiscard]] bool hasNormal() const;

  // --- Attribute access (const) ---
  float intensity() const;
  float time() const;
  uint16_t ring() const;
  const Color& color() const;
  const Label& label() const;
  const Eigen::Vector3f& normal() const;

  // --- Attribute access (mutable, only when IsConst=false) ---
  template <bool C = IsConst, typename = std::enable_if_t<!C>>
  float& intensity();

  template <bool C = IsConst, typename = std::enable_if_t<!C>>
  float& time();

  template <bool C = IsConst, typename = std::enable_if_t<!C>>
  uint16_t& ring();

  template <bool C = IsConst, typename = std::enable_if_t<!C>>
  Color& color();

  template <bool C = IsConst, typename = std::enable_if_t<!C>>
  Label& label();

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
// ArrowProxyImpl - Proxy for iterator's operator-> support
// =============================================================================
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
  using pointer = ArrowProxyImpl<IsConst>;
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
  pointer operator->() const { return ArrowProxyImpl<IsConst>(*cloud_, idx_); }
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

}  // namespace nanopcl

// =============================================================================
// Implementation of PointRefImpl methods (requires complete PointCloud)
// =============================================================================
// Note: These are defined after PointCloud is complete.
// Include this file, then point_cloud.hpp, then the impl below will work.

#endif  // NANOPCL_CORE_POINT_ACCESSORS_HPP
