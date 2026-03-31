#include "crisp_controllers/utils/async_csv_logger.hpp"
#include "crisp_controllers/utils/rt_timing_stats.hpp"
#include <iomanip>
#include <iostream>
#include <chrono>
#include <thread>
#include <sched.h>

namespace crisp_controllers {

AsyncCSVLogger::AsyncCSVLogger(const std::string &controller_name,
                               rclcpp::Logger logger)
    : controller_name_(controller_name), logger_(logger) {}

AsyncCSVLogger::~AsyncCSVLogger() { close(); }

bool AsyncCSVLogger::initialize(const rclcpp::Time &start_time,
                                const ControllerLogDataInterface& header_generator) {
  start_time_ = start_time;

  const char *user_ws = std::getenv("USER_WS");
  if (!user_ws) {
    RCLCPP_ERROR(logger_, "USER_WS environment variable not set");
    return false;
  }

  // Create log file directory
  std::filesystem::path log_dir =
      std::string(user_ws) + "/crisp_controller_logs";
  std::filesystem::create_directories(log_dir);

  // Generate filename with timestamp
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream filename_stream;
  filename_stream << log_dir.string() << "/" << controller_name_
                  << "_async_log_"
                  << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
                  << ".csv";

  std::string log_filename = filename_stream.str();
  csv_file_.open(log_filename, std::ios::out);

  if (csv_file_.is_open()) {
    RCLCPP_INFO(logger_, "Async CSV logging enabled, writing to: %s",
                log_filename.c_str());
    RCLCPP_INFO(logger_, "Lock-free ring buffer size: %zu samples",
                RING_BUFFER_SIZE);

    // Write header immediately
    csv_file_ << header_generator.generateHeader() << '\n';
    csv_file_.flush();

    // Start the writer thread
    logging_enabled_ = true;
    shutdown_requested_ = false;
    write_index_ = 0;
    read_index_ = 0;
    writer_thread_ = std::thread(&AsyncCSVLogger::writerThread, this);

// Set thread priority (optional, may require permissions)
#ifdef __linux__
    struct sched_param param;
    param.sched_priority = WRITER_THREAD_PRIORITY;
    pthread_setschedparam(writer_thread_.native_handle(), SCHED_OTHER, &param);
#endif

    return true;
  } else {
    RCLCPP_ERROR(logger_, "Failed to open async CSV log file: %s",
                 log_filename.c_str());
    logging_enabled_ = false;
    return false;
  }
}

void AsyncCSVLogger::close() {
  if (logging_enabled_.load()) {
    // Signal shutdown
    shutdown_requested_ = true;

    // Wait for writer thread to finish
    if (writer_thread_.joinable()) {
      writer_thread_.join();
    }

    // Close file
    if (csv_file_.is_open()) {
      csv_file_.flush();
      csv_file_.close();

      RCLCPP_INFO(logger_,
                  "Async CSV log file closed. Total samples: %zu, Dropped: %zu "
                  "(%.2f%%)",
                  total_samples_.load(), dropped_samples_.load(),
                  total_samples_.load() > 0
                      ? 100.0 * dropped_samples_.load() / total_samples_.load()
                      : 0.0);
    }

    logging_enabled_ = false;
  }
}

void AsyncCSVLogger::logData(const ControllerLogDataInterface &data) {
  if (!logging_enabled_.load()) {
    return;
  }

  total_samples_++;

  // Generate CSV line from data
  std::string csv_line = data.generateLine();

  // Lock-free write to ring buffer
  size_t write_idx = write_index_.load(std::memory_order_relaxed);
  size_t next_write_idx = (write_idx + 1) & (RING_BUFFER_SIZE - 1);
  size_t read_idx = read_index_.load(std::memory_order_acquire);

  // Check if buffer is full
  if (next_write_idx == read_idx) {
    // Buffer full - drop this sample
    dropped_samples_++;
    return;
  }

  // Write data to buffer
  ring_buffer_[write_idx] = csv_line;

  // Update write index (release semantics ensures data is visible to reader)
  write_index_.store(next_write_idx, std::memory_order_release);
}

void AsyncCSVLogger::writerThread() {
  RCLCPP_INFO(logger_, "CSV writer thread started");

  size_t flush_counter = 0;
  constexpr size_t FLUSH_INTERVAL = 100;

  // Initialize timing statistics
  writer_stats_.last_stats_log = std::chrono::steady_clock::now();

  while (!shutdown_requested_.load()) {
    // Check if data is available (acquire semantics to see writer's data)
    size_t read_idx = read_index_.load(std::memory_order_relaxed);
    size_t write_idx = write_index_.load(std::memory_order_acquire);

    if (read_idx == write_idx) {
      // Buffer is empty, sleep briefly
      std::this_thread::sleep_for(std::chrono::microseconds(WRITER_SLEEP_US));
      continue;
    }

    // Time the write operations
    struct timespec write_start, write_end;
    clock_gettime(CLOCK_MONOTONIC, &write_start);

    size_t items_written = 0;
    // Process available data
    while (read_idx != write_idx) {
      csv_file_ << ring_buffer_[read_idx] << '\n';

      // Move to next item
      read_idx = (read_idx + 1) & (RING_BUFFER_SIZE - 1);
      items_written++;

      // Periodic flush for data safety
      if (++flush_counter >= FLUSH_INTERVAL) {
        csv_file_.flush();
        flush_counter = 0;
      }
    }

    clock_gettime(CLOCK_MONOTONIC, &write_end);
    long write_duration_us = timespec_diff_us(write_start, write_end);

    // Update statistics atomically
    writer_stats_.write_count.fetch_add(items_written, std::memory_order_relaxed);
    writer_stats_.total_write_us.fetch_add(write_duration_us, std::memory_order_relaxed);

    long current_max = writer_stats_.max_write_us.load(std::memory_order_relaxed);
    while (write_duration_us > current_max &&
           !writer_stats_.max_write_us.compare_exchange_weak(current_max, write_duration_us)) {
      // Retry if another thread updated max
    }

    // Update read index (release semantics to signal space available)
    read_index_.store(read_idx, std::memory_order_release);

    // Log statistics every 5 seconds (less frequent than main loop)
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
            now - writer_stats_.last_stats_log).count() > 5000) {

      size_t count = writer_stats_.write_count.load();
      long total = writer_stats_.total_write_us.load();
      long max_write = writer_stats_.max_write_us.load();

      if (count > 0) {
        long avg = total / static_cast<long>(count);
        RCLCPP_INFO(logger_,
                    "CSV writer: avg=%ldus max=%ldus items=%zu queue_size=%zu dropped=%zu",
                    avg, max_write, count, getQueueSize(), dropped_samples_.load());
      }

      // Reset stats
      writer_stats_.write_count.store(0);
      writer_stats_.total_write_us.store(0);
      writer_stats_.max_write_us.store(0);
      writer_stats_.last_stats_log = now;
    }
  }

  // Process any remaining data before shutdown
  size_t read_idx = read_index_.load(std::memory_order_relaxed);
  size_t write_idx = write_index_.load(std::memory_order_acquire);

  while (read_idx != write_idx) {
    csv_file_ << ring_buffer_[read_idx] << '\n';
    read_idx = (read_idx + 1) & (RING_BUFFER_SIZE - 1);
  }

  csv_file_.flush();
  RCLCPP_INFO(logger_, "CSV writer thread finished");
}

} // namespace crisp_controllers
