#ifndef LOGGER_LOGGER_H
#define LOGGER_LOGGER_H

#include <chrono>
#include <iostream>
#include <sstream>
#include <unordered_map>

namespace logger {

// ANSI color codes
namespace color {
constexpr auto RESET = "\033[0m";
constexpr auto BOLD = "\033[1m";     // Bold text
constexpr auto DIM = "\033[2m";      // DEBUG - gray/dim
constexpr auto GREEN = "\033[32m";   // NOTICE - green
constexpr auto YELLOW = "\033[33m";  // WARN - yellow
constexpr auto RED = "\033[31m";     // ERROR - red
constexpr auto CYAN = "\033[36m";    // BENCH - cyan
// Combined bold + color codes
constexpr auto BOLD_GREEN = "\033[1;32m";
constexpr auto BOLD_YELLOW = "\033[1;33m";
constexpr auto BOLD_RED = "\033[1;31m";
constexpr auto BOLD_CYAN = "\033[1;36m";
constexpr auto BOLD_DIM = "\033[1;2m";
}  // namespace color

enum Level { DEBUG, INFO, WARN, ERROR, BENCH };

namespace detail {
inline Level min_level_ = INFO;

struct ThrottleInfo {
  std::chrono::steady_clock::time_point last_logged;
  size_t suppressed_count = 0;
};
inline std::unordered_map<std::string, ThrottleInfo> throttle_map_;
}  // namespace detail

inline void setLevel(Level level) { detail::min_level_ = level; }
inline Level getLevel() { return detail::min_level_; }

inline void setLevelFromString(const std::string &level_str) {
  std::string upper = level_str;
  std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);

  if (upper == "DEBUG")
    setLevel(DEBUG);
  else if (upper == "INFO")
    setLevel(INFO);
  else if (upper == "WARN" || upper == "WARNING")
    setLevel(WARN);
  else if (upper == "ERROR")
    setLevel(ERROR);
  else
    setLevel(INFO);  // default to INFO for invalid input
}

inline void log(Level level, const std::string &module,
                const std::string &msg) {
  // BENCH is always shown regardless of min_level
  if (level < detail::min_level_ && level != BENCH) return;

  const char *color_code = "";
  const char *bold_color = "";
  switch (level) {
    case DEBUG:
      color_code = color::DIM;
      bold_color = color::BOLD_DIM;
      break;
    case INFO:
      color_code = "";  // Plain text for INFO
      bold_color = color::BOLD;
      break;
    case WARN:
      color_code = color::YELLOW;
      bold_color = color::BOLD_YELLOW;
      break;
    case ERROR:
      color_code = color::RED;
      bold_color = color::BOLD_RED;
      break;
    case BENCH:
      color_code = color::CYAN;
      bold_color = color::BOLD_CYAN;
      break;
  }

  // If module is empty, just print message
  if (module.empty()) {
    std::cout << color_code << msg << color::RESET << "\n";
  } else {
    // Bold colored tag, then regular colored message
    std::cout << bold_color << "[" << module << "]:" << color::RESET << " "
              << color_code << msg << color::RESET << "\n";
  }
}

inline void bench(const std::string &msg) {
  std::cout << color::CYAN << msg << color::RESET << "\n";
}

inline void notice(const std::string &msg, bool bold = false) {
  if (bold) {
    std::cout << color::BOLD_GREEN << msg << color::RESET << "\n";
  } else {
    std::cout << color::GREEN << msg << color::RESET << "\n";
  }
}

template <typename... Args>
inline std::string fmt(Args... args) {
  std::ostringstream oss;
  ((oss << args), ...);
  return oss.str();
}

inline bool shouldLogThrottled(const std::string &key, double period_seconds) {
  auto now = std::chrono::steady_clock::now();

  // Check if this key exists
  auto it = detail::throttle_map_.find(key);
  if (it == detail::throttle_map_.end()) {
    // First time - create entry and log
    detail::throttle_map_[key] = detail::ThrottleInfo{now, 0};
    return true;
  }

  auto &info = it->second;
  auto elapsed = std::chrono::duration<double>(now - info.last_logged).count();

  if (elapsed >= period_seconds) {
    // Log suppressed count if any
    if (info.suppressed_count > 0) {
      std::cout << color::DIM << "[Suppressed " << info.suppressed_count
                << " similar messages]" << color::RESET << "\n";
    }
    info.last_logged = now;
    info.suppressed_count = 0;
    return true;
  } else {
    info.suppressed_count++;
    return false;
  }
}

// Clean macros - module name and variadic args
#define LOG_DEBUG(module, ...)             \
  if (logger::getLevel() <= logger::DEBUG) \
  logger::log(logger::DEBUG, module, logger::fmt(__VA_ARGS__))

#define LOG_INFO(module, ...) \
  logger::log(logger::INFO, module, logger::fmt(__VA_ARGS__))

#define LOG_WARN(module, ...) \
  logger::log(logger::WARN, module, logger::fmt(__VA_ARGS__))

#define LOG_ERROR(module, ...) \
  logger::log(logger::ERROR, module, logger::fmt(__VA_ARGS__))

// BENCH macro - no module parameter, just like NOTICE
#define LOG_BENCH(...) logger::bench(logger::fmt(__VA_ARGS__))

// NOTICE macros - no module parameter needed
#define LOG_NOTICE(...) logger::notice(logger::fmt(__VA_ARGS__), false)

#define LOG_NOTICE_BOLD(...) logger::notice(logger::fmt(__VA_ARGS__), true)

// Throttled logging macros - only log if period has elapsed
#define LOG_ERROR_THROTTLE(period, module, ...)                     \
  do {                                                              \
    static std::string _throttle_key =                              \
        std::string(__FILE__) + ":" + std::to_string(__LINE__);     \
    if (logger::shouldLogThrottled(_throttle_key, period)) {        \
      logger::log(logger::ERROR, module, logger::fmt(__VA_ARGS__)); \
    }                                                               \
  } while (0)

#define LOG_WARN_THROTTLE(period, module, ...)                     \
  do {                                                             \
    static std::string _throttle_key =                             \
        std::string(__FILE__) + ":" + std::to_string(__LINE__);    \
    if (logger::shouldLogThrottled(_throttle_key, period)) {       \
      logger::log(logger::WARN, module, logger::fmt(__VA_ARGS__)); \
    }                                                              \
  } while (0)

#define LOG_INFO_THROTTLE(period, module, ...)                     \
  do {                                                             \
    static std::string _throttle_key =                             \
        std::string(__FILE__) + ":" + std::to_string(__LINE__);    \
    if (logger::shouldLogThrottled(_throttle_key, period)) {       \
      logger::log(logger::INFO, module, logger::fmt(__VA_ARGS__)); \
    }                                                              \
  } while (0)

#define LOG_DEBUG_THROTTLE(period, module, ...)                       \
  do {                                                                \
    if (logger::getLevel() <= logger::DEBUG) {                        \
      static std::string _throttle_key =                              \
          std::string(__FILE__) + ":" + std::to_string(__LINE__);     \
      if (logger::shouldLogThrottled(_throttle_key, period)) {        \
        logger::log(logger::DEBUG, module, logger::fmt(__VA_ARGS__)); \
      }                                                               \
    }                                                                 \
  } while (0)

// "ONCE" logging macros - logs only once per program execution
// Uses very large throttle period (1 million seconds ~ 11.5 days)
#define LOG_ERROR_ONCE(module, ...)                                 \
  do {                                                              \
    static std::string _throttle_key =                              \
        std::string(__FILE__) + ":" + std::to_string(__LINE__);     \
    if (logger::shouldLogThrottled(_throttle_key, 1000000.0)) {     \
      logger::log(logger::ERROR, module, logger::fmt(__VA_ARGS__)); \
    }                                                               \
  } while (0)

#define LOG_WARN_ONCE(module, ...)                                 \
  do {                                                             \
    static std::string _throttle_key =                             \
        std::string(__FILE__) + ":" + std::to_string(__LINE__);    \
    if (logger::shouldLogThrottled(_throttle_key, 1000000.0)) {    \
      logger::log(logger::WARN, module, logger::fmt(__VA_ARGS__)); \
    }                                                              \
  } while (0)

#define LOG_INFO_ONCE(module, ...)                                 \
  do {                                                             \
    static std::string _throttle_key =                             \
        std::string(__FILE__) + ":" + std::to_string(__LINE__);    \
    if (logger::shouldLogThrottled(_throttle_key, 1000000.0)) {    \
      logger::log(logger::INFO, module, logger::fmt(__VA_ARGS__)); \
    }                                                              \
  } while (0)

#define LOG_DEBUG_ONCE(module, ...)                                   \
  do {                                                                \
    if (logger::getLevel() <= logger::DEBUG) {                        \
      static std::string _throttle_key =                              \
          std::string(__FILE__) + ":" + std::to_string(__LINE__);     \
      if (logger::shouldLogThrottled(_throttle_key, 1000000.0)) {     \
        logger::log(logger::DEBUG, module, logger::fmt(__VA_ARGS__)); \
      }                                                               \
    }                                                                 \
  } while (0)

}  // namespace logger

#endif  // LOGGER_LOGGER_H