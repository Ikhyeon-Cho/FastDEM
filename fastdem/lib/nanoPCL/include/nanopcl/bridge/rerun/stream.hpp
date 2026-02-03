// nanoPCL - Rerun visualization stream management
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_VISUALIZATION_RERUN_STREAM_HPP
#define NANOPCL_VISUALIZATION_RERUN_STREAM_HPP

#ifdef NANOPCL_USE_RERUN

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include <rerun.hpp>

namespace nanopcl::rr {

namespace detail {

/// Check if 'rerun' command is available in PATH
inline bool isViewerAvailable() {
#ifdef _WIN32
  return std::system("where rerun >nul 2>&1") == 0;
#else
  return std::system("command -v rerun >/dev/null 2>&1") == 0;
#endif
}

/// Print helpful error message when viewer is not found
inline void printViewerNotFoundError() {
  std::cerr << "\n"
            << "[nanoPCL] Error: Rerun Viewer not found in PATH.\n"
            << "[nanoPCL] \n"
            << "[nanoPCL] To visualize, please install the Rerun Viewer:\n"
            << "[nanoPCL]   - pip install rerun-sdk\n"
            << "[nanoPCL]   - or download from https://rerun.io/docs/getting-started/installing-viewer\n"
            << "[nanoPCL] \n"
            << "[nanoPCL] Alternatively, save to file:\n"
            << "[nanoPCL]   viz::init(\"app\", viz::ConnectMode::SAVE, \"output.rrd\");\n"
            << "[nanoPCL] \n";
}

} // namespace detail

/// Connection mode for Rerun viewer
enum class ConnectMode {
  SPAWN,   ///< Launch a new Rerun viewer process
  CONNECT, ///< Connect to an existing viewer (TCP address)
  SAVE,    ///< Save to .rrd file for later viewing
  SERVE    ///< Serve over HTTP for remote viewer access
};

/// Manages the global Rerun RecordingStream singleton
class StreamManager {
public:
  static StreamManager& instance() {
    static StreamManager mgr;
    return mgr;
  }

  /// Initialize the recording stream
  /// @param app_id Application identifier shown in Rerun viewer
  /// @param mode Connection mode (spawn, connect, save, or serve)
  /// @param addr_or_path TCP address (for CONNECT) or file path (for SAVE)
  /// @note Set RERUN_SERVE=1 to enable serve mode for remote viewer access.
  ///       Example: RERUN_SERVE=1 ./program
  ///       Then connect from remote: http://<this_machine_ip>:9876
  void init(const std::string& app_id = "nanoPCL",
            ConnectMode mode = ConnectMode::SPAWN,
            const std::string& addr_or_path = "") {
    rec_ = std::make_unique<rerun::RecordingStream>(app_id);

    // Check for RERUN_SERVE environment variable
    if (const char* env_serve = std::getenv("RERUN_SERVE")) {
      if (env_serve[0] != '\0' && env_serve[0] != '0') {
        mode = ConnectMode::SERVE;
      }
    }

    switch (mode) {
    case ConnectMode::SPAWN:
      // Check if viewer is available before attempting to spawn
      if (!detail::isViewerAvailable()) {
        detail::printViewerNotFoundError();
      }
      rec_->spawn().exit_on_failure();
      break;
    case ConnectMode::CONNECT:
      if (addr_or_path.empty()) {
        rec_->connect_grpc().exit_on_failure();
      } else {
        rec_->connect_grpc(addr_or_path).exit_on_failure();
      }
      break;
    case ConnectMode::SAVE:
      rec_->save(addr_or_path).exit_on_failure();
      break;
    case ConnectMode::SERVE: {
      auto result = rec_->serve_grpc();
      if (result.is_err()) {
        std::cerr << "[nanoPCL] Failed to start gRPC server\n";
      } else {
        std::cout << "[nanoPCL] Serving at: " << result.value << "\n";
        std::cout << "[nanoPCL] Connect with: rerun " << result.value << "\n";
      }
      break;
    }
    }

    initialized_ = true;
  }

  /// Get the recording stream (lazy initialization with defaults)
  rerun::RecordingStream& stream() {
    if (!initialized_) {
      init();
    }
    return *rec_;
  }

  /// Check if the stream has been initialized
  bool isInitialized() const { return initialized_; }

private:
  StreamManager() = default;
  StreamManager(const StreamManager&) = delete;
  StreamManager& operator=(const StreamManager&) = delete;

  std::unique_ptr<rerun::RecordingStream> rec_;
  bool initialized_ = false;
};

/// Get the global recording stream
inline rerun::RecordingStream& stream() {
  return StreamManager::instance().stream();
}

/// Initialize the visualization stream
/// @param app_id Application identifier shown in Rerun viewer
/// @param mode Connection mode (spawn, connect, or save)
/// @param addr_or_path TCP address (for CONNECT) or file path (for SAVE)
inline void init(const std::string& app_id = "nanoPCL",
                 ConnectMode mode = ConnectMode::SPAWN,
                 const std::string& addr_or_path = "") {
  StreamManager::instance().init(app_id, mode, addr_or_path);
}

} // namespace nanopcl::rr

#endif // NANOPCL_USE_RERUN

#endif // NANOPCL_VISUALIZATION_RERUN_STREAM_HPP
