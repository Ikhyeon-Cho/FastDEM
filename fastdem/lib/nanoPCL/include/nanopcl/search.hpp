// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Search module: Spatial data structures for efficient nearest neighbor
// queries.
//
// Available searchers:
//   - VoxelHash: Hash-based spatial index O(1) (fast build, good for radius
//   search)
//   - KdTree:    KD-tree index O(log N) (slower build, best for KNN search)

#ifndef NANOPCL_SEARCH_HPP
#define NANOPCL_SEARCH_HPP

#include "nanopcl/search/kdtree.hpp"
#include "nanopcl/search/voxel_hash.hpp"

#endif // NANOPCL_SEARCH_HPP
