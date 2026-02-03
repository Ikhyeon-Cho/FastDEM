// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * point_types.hpp
 *
 * Point cloud type aliases for fastdem.
 */

#ifndef FASTDEM_POINT_TYPES_HPP
#define FASTDEM_POINT_TYPES_HPP

#include <nanopcl/core.hpp>

namespace fastdem {

using PointCloud = nanopcl::PointCloud;
using Point = nanopcl::Point;
using Color = nanopcl::Color;

}  // namespace fastdem

#endif  // FASTDEM_POINT_TYPES_HPP
