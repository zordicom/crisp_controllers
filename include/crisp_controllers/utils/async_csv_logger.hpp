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
#include "crisp_controllers/utils/controller_log_data.hpp"

namespace crisp_controllers {

/**
 * @brief Asynchronous CSV logger that writes data in a background thread
 *
 * This class uses a circular buffer with atomic operations to minimize
 * blocking in the real-time control thread. Data is copied to a queue
 * and written to disk by a background thread.
 */
class AsyncCSVLogger {
public:
  AsyncCSVLogger(const std::string& controller_name, rclcpp::Logger logger);
  ~AsyncCSVLogger();

  bool initialize(size_t num_joints, const rclcpp::Time& start_time);

  /**
   * @brief Queue data for logging (non-blocking for real-time thread)
   *
   * This method copies the data to a lock-free circular buffer.
   * If the buffer is full, the oldest data is overwritten (data loss).
   * Returns immediately without blocking.
   */
  void logData(const ControllerLogData& data, const rclcpp::Time& current_time);

  void close();
  bool isLoggingEnabled() const { return logging_enabled_.load(); }

  /**
   * @brief Get number of dropped samples due to buffer overflow
   */
  size_t getDroppedSamples() const { return dropped_samples_.load(); }

  /**
   * @brief Get current queue size
   */
  size_t getQueueSize() const { return queue_size_.load(); }

private:
  void writerThread();
  void writeHeader(size_t num_joints);
  void writePose(const pinocchio::SE3& pose);
  void writeRPY(const pinocchio::SE3& pose);
  void processLogData(const ControllerLogData& data);

  // Configuration
  std::string controller_name_;
  rclcpp::Logger logger_;
  size_t num_joints_ = 0;
  rclcpp::Time start_time_;

  // Thread management
  std::thread writer_thread_;
  std::atomic<bool> logging_enabled_{false};
  std::atomic<bool> shutdown_requested_{false};

  // Data queue with mutex for simplicity (can upgrade to lock-free queue if needed)
  // Using a simple mutex-based queue initially, as std::deque with mutex
  // is often fast enough and easier to maintain than a full lock-free implementation
  mutable std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<ControllerLogData> data_queue_;

  // Statistics
  std::atomic<size_t> queue_size_{0};
  std::atomic<size_t> dropped_samples_{0};
  std::atomic<size_t> total_samples_{0};

  // File I/O
  std::ofstream csv_file_;

  // Configuration parameters
  static constexpr size_t MAX_QUEUE_SIZE = 1000;  // Maximum buffered samples
  static constexpr size_t BATCH_WRITE_SIZE = 10;  // Write in batches for efficiency
  static constexpr int WRITER_THREAD_PRIORITY = 10; // Lower priority than control thread
};

} // namespace crisp_controllers