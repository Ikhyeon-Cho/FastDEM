// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// C++17 compatible Span (lightweight view over contiguous data)
//
// This is a minimal implementation for C++17 environments.
// For C++20+, prefer std::span instead.

#ifndef NANOPCL_CORE_SPAN_HPP
#define NANOPCL_CORE_SPAN_HPP

#include <cstddef>
#include <vector>

namespace npcl {

/**
 * @brief Non-owning view over a contiguous sequence of elements
 *
 * Lightweight alternative to std::span for C++17 compatibility.
 * Provides safe iteration without copying data.
 *
 * @tparam T Element type (typically const-qualified for read-only access)
 *
 * Example usage:
 * @code
 *   std::vector<int> data = {1, 2, 3, 4, 5};
 *   Span<const int> view(data.data() + 1, 3);  // [2, 3, 4]
 *
 *   for (int val : view) {
 *     std::cout << val << "\n";
 *   }
 * @endcode
 */
template <typename T>
class Span {
 public:
  // ==========================================================================
  // Type Aliases
  // ==========================================================================

  using element_type = T;
  using value_type = std::remove_cv_t<T>;
  using size_type = std::size_t;
  using difference_type = std::ptrdiff_t;
  using pointer = T*;
  using const_pointer = const T*;
  using reference = T&;
  using const_reference = const T&;
  using iterator = pointer;
  using const_iterator = const_pointer;

  // ==========================================================================
  // Constructors
  // ==========================================================================

  /// Default constructor: empty span
  constexpr Span() noexcept : data_(nullptr), size_(0) {}

  /// Construct from pointer and size
  constexpr Span(pointer ptr, size_type count) noexcept
      : data_(ptr), size_(count) {}

  /// Construct from two pointers [first, last)
  constexpr Span(pointer first, pointer last) noexcept
      : data_(first), size_(static_cast<size_type>(last - first)) {}

  /// Construct from std::vector (implicit conversion)
  template <typename Allocator>
  Span(const std::vector<value_type, Allocator>& vec) noexcept
      : data_(vec.data()), size_(vec.size()) {}

  /// Construct from mutable std::vector
  template <typename Allocator>
  Span(std::vector<value_type, Allocator>& vec) noexcept
      : data_(vec.data()), size_(vec.size()) {}

  /// Construct from C-style array
  template <std::size_t N>
  constexpr Span(T (&arr)[N]) noexcept : data_(arr), size_(N) {}

  // ==========================================================================
  // Iterators (Range-based for loop support)
  // ==========================================================================

  constexpr iterator begin() const noexcept { return data_; }
  constexpr iterator end() const noexcept { return data_ + size_; }
  constexpr const_iterator cbegin() const noexcept { return data_; }
  constexpr const_iterator cend() const noexcept { return data_ + size_; }

  // ==========================================================================
  // Element Access
  // ==========================================================================

  /// Access element by index (no bounds checking)
  constexpr reference operator[](size_type idx) const { return data_[idx]; }

  /// Access first element
  constexpr reference front() const { return data_[0]; }

  /// Access last element
  constexpr reference back() const { return data_[size_ - 1]; }

  /// Get pointer to underlying data
  constexpr pointer data() const noexcept { return data_; }

  // ==========================================================================
  // Observers
  // ==========================================================================

  /// Number of elements
  constexpr size_type size() const noexcept { return size_; }

  /// Size in bytes
  constexpr size_type size_bytes() const noexcept { return size_ * sizeof(T); }

  /// Check if empty
  [[nodiscard]] constexpr bool empty() const noexcept { return size_ == 0; }

  // ==========================================================================
  // Subviews
  // ==========================================================================

  /// First count elements
  constexpr Span<T> first(size_type count) const {
    return Span<T>(data_, count);
  }

  /// Last count elements
  constexpr Span<T> last(size_type count) const {
    return Span<T>(data_ + (size_ - count), count);
  }

  /// Subspan starting at offset with count elements
  constexpr Span<T> subspan(
      size_type offset, size_type count = static_cast<size_type>(-1)) const {
    if (count == static_cast<size_type>(-1)) {
      count = size_ - offset;
    }
    return Span<T>(data_ + offset, count);
  }

 private:
  pointer data_;
  size_type size_;
};

// Deduction guides (C++17)
template <typename T, std::size_t N>
Span(T (&)[N]) -> Span<T>;

template <typename Container>
Span(const Container&) -> Span<const typename Container::value_type>;

template <typename Container>
Span(Container&) -> Span<typename Container::value_type>;

}  // namespace npcl

#endif  // NANOPCL_CORE_SPAN_HPP
