#include "crisp_controllers/utils/csv_logger.hpp"
#include <iomanip>
#include <iostream>

namespace crisp_controllers {

ControllerCSVLogger::ControllerCSVLogger(const std::string &controller_name,
                                         rclcpp::Logger logger)
    : controller_name_(controller_name), logger_(logger) {}

ControllerCSVLogger::~ControllerCSVLogger() { close(); }

bool ControllerCSVLogger::initialize(const rclcpp::Time &start_time,
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
  filename_stream << log_dir.string() << "/" << controller_name_ << "_log_"
                  << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
                  << ".csv";

  std::string log_filename = filename_stream.str();
  csv_file_.open(log_filename, std::ios::out);

  if (csv_file_.is_open()) {
    logging_enabled_ = true;
    RCLCPP_INFO(logger_, "CSV logging enabled, writing to: %s",
                log_filename.c_str());

    // Write header immediately
    csv_file_ << header_generator.generateHeader() << '\n';
    csv_file_.flush();

    return true;
  } else {
    RCLCPP_ERROR(logger_, "Failed to open CSV log file: %s",
                 log_filename.c_str());
    logging_enabled_ = false;
    return false;
  }
}

void ControllerCSVLogger::logData(const ControllerLogDataInterface &data) {
  if (!logging_enabled_ || !csv_file_.is_open()) {
    return;
  }

  // Write CSV line
  csv_file_ << data.generateLine() << '\n';

  // Periodic flush
  if (++flush_counter_ >= FLUSH_INTERVAL) {
    csv_file_.flush();
    flush_counter_ = 0;
  }
}

void ControllerCSVLogger::close() {
  if (csv_file_.is_open()) {
    csv_file_.flush();
    csv_file_.close();
    RCLCPP_INFO(logger_, "CSV log file closed.");
  }
  logging_enabled_ = false;
}

} // namespace crisp_controllers
