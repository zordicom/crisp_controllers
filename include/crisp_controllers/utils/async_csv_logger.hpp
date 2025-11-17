#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <pinocchio/spatial/se3.hpp>
#include "crisp_controllers/utils/controller_log_data_interface.hpp"
#include "crisp_controllers/utils/csv_logger_interface.hpp"

namespace crisp_controllers {

/**
 * @brief Asynchronous CSV logger that writes data in a background thread
 *
 * This class uses a circular buffer with atomic operations to minimize
 * blocking in the real-time control thread. Data is copied to a queue
 * and written to disk by a background thread.
 */
class AsyncCSVLogger : public CSVLoggerInterface {
public:
  AsyncCSVLogger(const std::string& controller_name, rclcpp::Logger logger);
  ~AsyncCSVLogger() override;

  bool initialize(const rclcpp::Time& start_time,
                 const ControllerLogDataInterface& header_generator) override;

  /**
   * @brief Queue data for logging (non-blocking for real-time thread)
   *
   * This method copies the data string to a queue.
   * If the buffer is full, the oldest data is overwritten (data loss).
   * Returns immediately without blocking.
   */
  void logData(const ControllerLogDataInterface& data) override;

  void close() override;
  bool isLoggingEnabled() const override { return logging_enabled_.load(); }

  /**
   * @brief Get number of dropped samples due to buffer overflow
   */
  size_t getDroppedSamples() const { return dropped_samples_.load(); }

  /**
   * @brief Get current queue size
   */
  size_t getQueueSize() const {
    size_t write_idx = write_index_.load(std::memory_order_relaxed);
    size_t read_idx = read_index_.load(std::memory_order_relaxed);
    return (write_idx - read_idx) & (RING_BUFFER_SIZE - 1);
  }

private:
  void writerThread();

  // Configuration
  std::string controller_name_;
  rclcpp::Logger logger_;
  rclcpp::Time start_time_;

  // Thread management
  std::thread writer_thread_;
  std::atomic<bool> logging_enabled_{false};
  std::atomic<bool> shutdown_requested_{false};

  // Lock-free ring buffer for CSV strings
  // Each element stores a pre-formatted CSV line
  static constexpr size_t RING_BUFFER_SIZE = 1024;  // Must be power of 2
  std::array<std::string, RING_BUFFER_SIZE> ring_buffer_;
  std::atomic<size_t> write_index_{0};
  std::atomic<size_t> read_index_{0};

  // Statistics
  std::atomic<size_t> dropped_samples_{0};
  std::atomic<size_t> total_samples_{0};

  // File I/O
  std::ofstream csv_file_;

  // Configuration parameters
  static constexpr int WRITER_THREAD_PRIORITY = 10; // Lower priority than control thread
  static constexpr int WRITER_SLEEP_US = 100; // Sleep time when buffer is empty (microseconds)
};

} // namespace crisp_controllers