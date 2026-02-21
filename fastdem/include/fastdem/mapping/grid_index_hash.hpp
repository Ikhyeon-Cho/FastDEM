// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * grid_index_hash.hpp
 *
 * Hash utilities for grid_map::Index used as unordered_map keys.
 *
 *  Created on: Feb 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FASTDEM_MAPPING_GRID_INDEX_HASH_HPP
#define FASTDEM_MAPPING_GRID_INDEX_HASH_HPP

#include <cstdint>
#include <functional>
#include <unordered_map>

#include <grid_map_core/TypeDefs.hpp>

namespace fastdem {

struct IndexHash {
  std::size_t operator()(const grid_map::Index& idx) const {
    const auto key =
        (static_cast<uint64_t>(static_cast<uint32_t>(idx(0))) << 32) |
        static_cast<uint64_t>(static_cast<uint32_t>(idx(1)));
    return std::hash<uint64_t>()(key);
  }
};

struct IndexEqual {
  bool operator()(const grid_map::Index& a, const grid_map::Index& b) const {
    return a(0) == b(0) && a(1) == b(1);
  }
};

template <typename T>
using CellMap = std::unordered_map<grid_map::Index, T, IndexHash, IndexEqual>;

}  // namespace fastdem

#endif  // FASTDEM_MAPPING_GRID_INDEX_HASH_HPP
