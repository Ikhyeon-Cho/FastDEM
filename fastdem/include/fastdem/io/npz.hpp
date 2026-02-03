// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * npz.hpp
 *
 * NumPy .npz format for lossless ElevationMap serialization.
 * Compatible with numpy.load() / numpy.savez() in Python.
 *
 *  Created on: Feb 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FASTDEM_IO_NPZ_HPP
#define FASTDEM_IO_NPZ_HPP

#include <string>
#include <vector>

#include "fastdem/elevation_map.hpp"

namespace fastdem {
namespace io {

/// Save all layers + metadata as NumPy .npz archive.
bool saveNpz(const std::string& filename, const ElevationMap& map);

/// Save specific layers + metadata as NumPy .npz archive.
bool saveNpz(const std::string& filename, const ElevationMap& map,
             const std::vector<std::string>& layer_names);

/// Load ElevationMap from .npz archive (saved by saveNpz).
bool loadNpz(const std::string& filename, ElevationMap& map);

}  // namespace io
}  // namespace fastdem

#endif  // FASTDEM_IO_NPZ_HPP
