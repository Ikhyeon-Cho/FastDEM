# Performance Guide

## Bulk Insert: add() vs resize()

When inserting large amounts of data, `resize() + index` is 2-3x faster than `add()`.

```cpp
// Method A: add() - convenient, slower for bulk
cloud.reserve(N);
for (size_t i = 0; i < N; ++i) {
    cloud.add(PointXYZI{...});  // Capacity check each iteration
}

// Method B: resize() + index - fast for bulk
cloud.enableIntensity();
cloud.resize(N);
auto& xyz = cloud.xyz();
auto& intensity = cloud.intensity();
for (size_t i = 0; i < N; ++i) {
    xyz[i] = Point(x, y, z);
    intensity[i] = val;
}

// Method C: resize() + PointRef - fast AND convenient
cloud.enableIntensity();
cloud.resize(N);
for (size_t i = 0; i < N; ++i) {
    auto pt = cloud.point(i);
    pt.point() = Point(x, y, z);
    pt.intensity() = val;
}
```

**Benchmark (1M points):**
| Method | Time |
|--------|------|
| add() with reserve | ~2.7 ms |
| resize() + index | ~1.0 ms |
| resize() + PointRef | ~1.0 ms |

## Memory Management

### reserve() Before Bulk Operations

```cpp
// BAD: Multiple reallocations
PointCloud cloud;
for (int i = 0; i < 100000; ++i) {
    cloud.add(Point(i, 0, 0));  // Vector may reallocate
}

// GOOD: Zero reallocations
PointCloud cloud;
cloud.reserve(100000);
for (int i = 0; i < 100000; ++i) {
    cloud.add(Point(i, 0, 0));
}
```

### shrink_to_fit() After Filtering

```cpp
PointCloud filtered = cloud[keep_indices];
filtered.shrink_to_fit();  // Release unused capacity
```

## Filtering: O(N) vs O(N²)

**Never use erase() in a loop!**

```cpp
// BAD: O(N²) - each erase shifts all elements
for (auto it = cloud.begin(); it != cloud.end();) {
    if (shouldRemove(*it)) {
        it = cloud.erase(it);  // O(N) per call!
    } else {
        ++it;
    }
}

// GOOD: O(N) - collect indices, extract once
std::vector<size_t> keep;
keep.reserve(cloud.size());
for (size_t i = 0; i < cloud.size(); ++i) {
    if (!shouldRemove(cloud.point(i))) {
        keep.push_back(i);
    }
}
PointCloud filtered = cloud[keep];  // O(keep.size())
```

**Benchmark (10k points, remove 50%):**
| Method | Time |
|--------|------|
| erase() loop | ~15,000 µs |
| cloud[indices] | ~150 µs |

## Channel Access Patterns

### Single Channel: Use Direct Access

```cpp
// Cache-friendly, SIMD-vectorizable
std::vector<float>& intensities = cloud.intensity();
float sum = 0;
for (float i : intensities) {
    sum += i;
}
```

### Multiple Channels: Use PointRef

```cpp
// Convenient when you need several attributes
for (size_t i = 0; i < cloud.size(); ++i) {
    auto pt = cloud.point(i);
    if (pt.ring() < 8 && pt.intensity() > threshold) {
        process(pt);
    }
}
```

## Summary

| Scenario | Recommendation |
|----------|----------------|
| Small inserts (< 1000) | `add(PointXYZI{...})` |
| Bulk inserts (> 10000) | `resize() + PointRef` |
| Process one channel | Direct access: `xyz()`, `intensity()` |
| Process multiple channels | `point(i)` or iterator |
| Filtering | `cloud[indices]` |
| Before bulk add | `reserve(n)` |
| After filtering | `shrink_to_fit()` |
