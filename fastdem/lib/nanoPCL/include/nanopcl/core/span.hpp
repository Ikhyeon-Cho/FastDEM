// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT
//
// Span: Non-owning view over contiguous data (C++17 std::span alternative)

#ifndef NANOPCL_CORE_SPAN_HPP
#define NANOPCL_CORE_SPAN_HPP

#include <cstddef>
#include <vector>

namespace nanopcl {

template <typename T>
class Span {
public:
  using size_type = std::size_t;
  using pointer = T*;
  using reference = T&;
  using iterator = pointer;

  // Constructors
  constexpr Span() noexcept
      : data_(nullptr), size_(0) {}
  constexpr Span(pointer ptr, size_type count) noexcept
      : data_(ptr), size_(count) {}

  template <typename Alloc>
  Span(std::vector<std::remove_cv_t<T>, Alloc>& vec) noexcept
      : data_(vec.data()), size_(vec.size()) {}

  template <typename Alloc>
  Span(const std::vector<std::remove_cv_t<T>, Alloc>& vec) noexcept
      : data_(vec.data()), size_(vec.size()) {}

  // Iterators
  constexpr iterator begin() const noexcept { return data_; }
  constexpr iterator end() const noexcept { return data_ + size_; }

  // Element access
  constexpr reference operator[](size_type i) const { return data_[i]; }
  constexpr pointer data() const noexcept { return data_; }

  // Observers
  constexpr size_type size() const noexcept { return size_; }
  constexpr bool empty() const noexcept { return size_ == 0; }

  // Subviews
  constexpr Span first(size_type count) const { return {data_, count}; }
  constexpr Span last(size_type count) const { return {data_ + size_ - count, count}; }
  constexpr Span subspan(size_type offset, size_type count) const {
    return {data_ + offset, count};
  }

private:
  pointer data_;
  size_type size_;
};

// Deduction guides
template <typename T, typename Alloc>
Span(std::vector<T, Alloc>&) -> Span<T>;

template <typename T, typename Alloc>
Span(const std::vector<T, Alloc>&) -> Span<const T>;

} // namespace nanopcl

#endif // NANOPCL_CORE_SPAN_HPP
