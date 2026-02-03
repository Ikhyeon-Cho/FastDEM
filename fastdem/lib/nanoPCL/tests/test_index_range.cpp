// nanoPCL - Test: IndexRange

#include <cassert>
#include <iostream>
#include <nanopcl/core/types.hpp>
#include <vector>

using namespace nanopcl;

#define TEST(name)                      \
  std::cout << "  " << #name << "... "; \
  test_##name();                        \
  std::cout << "OK\n"

#define ASSERT_EQ(a, b) assert((a) == (b))
#define ASSERT_TRUE(cond) assert(cond)
#define ASSERT_FALSE(cond) assert(!(cond))

// =============================================================================
// Basic Tests
// =============================================================================

void test_range_iteration() {
  std::vector<size_t> collected;

  for (auto i : IndexRange(5)) {
    collected.push_back(i);
  }

  ASSERT_EQ(collected.size(), 5u);
  ASSERT_EQ(collected[0], 0u);
  ASSERT_EQ(collected[1], 1u);
  ASSERT_EQ(collected[2], 2u);
  ASSERT_EQ(collected[3], 3u);
  ASSERT_EQ(collected[4], 4u);
}

void test_range_size() {
  IndexRange r1(0);
  IndexRange r2(10);
  IndexRange r3(1000);

  ASSERT_EQ(r1.size(), 0u);
  ASSERT_EQ(r2.size(), 10u);
  ASSERT_EQ(r3.size(), 1000u);
}

void test_range_empty() {
  std::vector<size_t> collected;

  for (auto i : IndexRange(0)) {
    collected.push_back(i);
  }

  ASSERT_TRUE(collected.empty());
}

void test_range_single() {
  std::vector<size_t> collected;

  for (auto i : IndexRange(1)) {
    collected.push_back(i);
  }

  ASSERT_EQ(collected.size(), 1u);
  ASSERT_EQ(collected[0], 0u);
}

void test_range_large() {
  size_t sum = 0;
  size_t n = 10000;

  for (auto i : IndexRange(n)) {
    sum += i;
  }

  // Sum of 0 to n-1 = n*(n-1)/2
  ASSERT_EQ(sum, n * (n - 1) / 2);
}

// =============================================================================
// Iterator Tests
// =============================================================================

void test_iterator_dereference() {
  IndexRange range(5);
  auto it = range.begin();

  ASSERT_EQ(*it, 0u);
  ++it;
  ASSERT_EQ(*it, 1u);
  ++it;
  ASSERT_EQ(*it, 2u);
}

void test_iterator_increment() {
  IndexRange range(3);
  auto it = range.begin();

  ASSERT_EQ(*it, 0u);
  auto& ref = ++it;
  ASSERT_EQ(*it, 1u);
  ASSERT_EQ(&ref, &it); // Returns reference to self
}

void test_iterator_comparison() {
  IndexRange range(5);

  auto begin = range.begin();
  auto end = range.end();

  ASSERT_TRUE(begin != end);

  // Advance to end
  for (int i = 0; i < 5; ++i) {
    ++begin;
  }

  ASSERT_FALSE(begin != end);
}

void test_iterator_begin_end() {
  IndexRange range(3);

  auto it = range.begin();
  auto end = range.end();

  std::vector<size_t> values;
  while (it != end) {
    values.push_back(*it);
    ++it;
  }

  ASSERT_EQ(values.size(), 3u);
  ASSERT_EQ(values[0], 0u);
  ASSERT_EQ(values[1], 1u);
  ASSERT_EQ(values[2], 2u);
}

// =============================================================================
// Constexpr Tests
// =============================================================================

void test_constexpr_construction() {
  constexpr IndexRange range(10);
  constexpr size_t sz = range.size();

  ASSERT_EQ(sz, 10u);
}

void test_constexpr_iterator() {
  constexpr IndexRange range(5);
  constexpr auto begin = range.begin();
  constexpr auto end = range.end();

  ASSERT_TRUE(begin != end);
}

// =============================================================================
// Use Case Tests
// =============================================================================

void test_pointcloud_iteration_pattern() {
  // Simulates typical usage: for (auto i : IndexRange(cloud.size()))
  std::vector<float> data = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};

  float sum = 0;
  for (auto i : IndexRange(data.size())) {
    sum += data[i];
  }

  ASSERT_EQ(sum, 15.0f);
}

void test_nested_iteration() {
  int count = 0;

  for (auto i : IndexRange(3)) {
    for (auto j : IndexRange(4)) {
      (void)i;
      (void)j;
      ++count;
    }
  }

  ASSERT_EQ(count, 12);
}

// =============================================================================
// Main
// =============================================================================

int main() {
  std::cout << "=== test_index_range ===\n";

  std::cout << "Basic:\n";
  TEST(range_iteration);
  TEST(range_size);
  TEST(range_empty);
  TEST(range_single);
  TEST(range_large);

  std::cout << "Iterator:\n";
  TEST(iterator_dereference);
  TEST(iterator_increment);
  TEST(iterator_comparison);
  TEST(iterator_begin_end);

  std::cout << "Constexpr:\n";
  TEST(constexpr_construction);
  TEST(constexpr_iterator);

  std::cout << "Use Cases:\n";
  TEST(pointcloud_iteration_pattern);
  TEST(nested_iteration);

  std::cout << "\nAll tests passed!\n";
  return 0;
}
