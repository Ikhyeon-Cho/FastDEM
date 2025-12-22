/*
 * log.h - Internal logging macros for ppl
 *
 * Thin wrapper around spdlog with PPL_ prefix to avoid name collisions.
 * For internal ppl use only.
 *
 * Created on: Dec 2024
 * Author: Ikhyeon Cho
 */

#ifndef PPL_DETAIL_LOG_H
#define PPL_DETAIL_LOG_H

#include <spdlog/spdlog.h>

#define PPL_LOG_DEBUG(...) spdlog::debug(__VA_ARGS__)
#define PPL_LOG_INFO(...) spdlog::info(__VA_ARGS__)
#define PPL_LOG_WARN(...) spdlog::warn(__VA_ARGS__)
#define PPL_LOG_ERROR(...) spdlog::error(__VA_ARGS__)

#endif  // PPL_DETAIL_LOG_H
