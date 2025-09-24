#pragma once
#include <iostream>
#include <sstream>

namespace logger {

// ANSI color codes
namespace color {
constexpr auto RESET = "\033[0m";
constexpr auto BOLD = "\033[1m";    // Bold text
constexpr auto DIM = "\033[2m";     // DEBUG - gray/dim
constexpr auto GREEN = "\033[32m";  // NOTICE - green
constexpr auto YELLOW = "\033[33m"; // WARN - yellow
constexpr auto RED = "\033[31m";    // ERROR - red
constexpr auto CYAN = "\033[36m";   // BENCH - cyan
// Combined bold + color codes
constexpr auto BOLD_GREEN = "\033[1;32m";
constexpr auto BOLD_YELLOW = "\033[1;33m";
constexpr auto BOLD_RED = "\033[1;31m";
constexpr auto BOLD_CYAN = "\033[1;36m";
constexpr auto BOLD_DIM = "\033[1;2m";
} // namespace color

enum Level { DEBUG, INFO, WARN, ERROR, BENCH };

class Logger {
  inline static Level min_level_ = INFO;
  inline static bool benchmark_enabled_ = true;

public:
  static void setLevel(Level level) { min_level_ = level; }
  static void setBenchmarkEnabled(bool enable) { benchmark_enabled_ = enable; }
  static Level getLevel() { return min_level_; }
  static bool isBenchmarkEnabled() { return benchmark_enabled_; }

  // Core log function
  static void log(Level level, const std::string &module,
                  const std::string &msg) {
    // BENCH is always shown regardless of min_level
    if (level < min_level_ && level != BENCH)
      return;
    if (level == BENCH && !benchmark_enabled_)
      return;

    const char *color = "";
    const char *bold_color = "";
    switch (level) {
    case DEBUG:
      color = color::DIM;
      bold_color = color::BOLD_DIM;
      break;
    case INFO:
      color = ""; // Plain text for INFO
      bold_color = color::BOLD;
      break;
    case WARN:
      color = color::YELLOW;
      bold_color = color::BOLD_YELLOW;
      break;
    case ERROR:
      color = color::RED;
      bold_color = color::BOLD_RED;
      break;
    case BENCH:
      color = color::CYAN;
      bold_color = color::BOLD_CYAN;
      break;
    }

    // If module is empty, just print message
    if (module.empty()) {
      std::cout << color << msg << color::RESET << "\n";
    } else {
      // Bold colored tag, then regular colored message
      std::cout << bold_color << "[" << module << "]:" << color::RESET << " "
                << color << msg << color::RESET << "\n";
    }
  }

  // Special function for NOTICE messages (no module parameter)
  // NOTICE uses green color and never has module tags
  static void notice(const std::string &msg, bool bold = false) {
    if (bold) {
      std::cout << color::BOLD_GREEN << msg << color::RESET << "\n";
    } else {
      std::cout << color::GREEN << msg << color::RESET << "\n";
    }
  }

  // Variadic concatenation helper using fold expression
  template <typename... Args> static std::string fmt(Args... args) {
    std::ostringstream oss;
    ((oss << args), ...);
    return oss.str();
  }
};

// Clean macros - module name and variadic args
#define LOG_DEBUG(module, ...)                                                 \
  if (logger::Logger::getLevel() <= logger::DEBUG)                             \
  logger::Logger::log(logger::DEBUG, module, logger::Logger::fmt(__VA_ARGS__))

#define LOG_INFO(module, ...)                                                  \
  logger::Logger::log(logger::INFO, module, logger::Logger::fmt(__VA_ARGS__))

#define LOG_WARN(module, ...)                                                  \
  logger::Logger::log(logger::WARN, module, logger::Logger::fmt(__VA_ARGS__))

#define LOG_ERROR(module, ...)                                                 \
  logger::Logger::log(logger::ERROR, module, logger::Logger::fmt(__VA_ARGS__))

#define LOG_BENCH(stage, ms)                                                   \
  logger::Logger::log(logger::BENCH, stage, logger::Logger::fmt(ms, " ms"))

// NOTICE macros - no module parameter needed
#define LOG_NOTICE(...)                                                        \
  logger::Logger::notice(logger::Logger::fmt(__VA_ARGS__), false)

#define LOG_NOTICE_BOLD(...)                                                   \
  logger::Logger::notice(logger::Logger::fmt(__VA_ARGS__), true)

} // namespace logger