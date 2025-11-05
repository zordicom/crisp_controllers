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

namespace crisp_controllers {

struct ControllerLogData {
  double timestamp;

  // Task space forces (6 DOF each)
  Eigen::Vector<double, 6> task_force_P;
  Eigen::Vector<double, 6> task_force_D;
  Eigen::Vector<double, 6> task_force_total;

  // Torque components (per joint)
  Eigen::VectorXd tau_task;
  Eigen::VectorXd tau_nullspace;
  Eigen::VectorXd tau_joint_limits;
  Eigen::VectorXd tau_friction;
  Eigen::VectorXd tau_coriolis;
  Eigen::VectorXd tau_gravity;
  Eigen::VectorXd tau_wrench;
  Eigen::VectorXd tau_total;

  // Error metrics
  Eigen::Vector<double, 6> error;
  double error_rot_magnitude;
  double error_pos_magnitude;

  // Poses
  pinocchio::SE3 current_pose;
  pinocchio::SE3 target_pose;

  // Stiffness and damping (diagonal elements)
  Eigen::Vector<double, 6> stiffness_diag;
  Eigen::Vector<double, 6> damping_diag;

  // Joint states
  Eigen::VectorXd q_raw;
  Eigen::VectorXd q_filtered;
  Eigen::VectorXd dq_raw;
  Eigen::VectorXd dq_filtered;
  Eigen::VectorXd q_goal;
  Eigen::VectorXd dq_goal;

  // Filter parameters
  double filter_q;
  double filter_dq;
  double filter_output_torque;

  // Timing information
  double loop_duration_ms;  // Control loop duration in milliseconds
};

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