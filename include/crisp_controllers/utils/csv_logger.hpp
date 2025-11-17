#pragma once

#include <fstream>
#include <filesystem>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <pinocchio/spatial/se3.hpp>
#include "crisp_controllers/utils/controller_log_data_interface.hpp"
#include "crisp_controllers/utils/csv_logger_interface.hpp"

namespace crisp_controllers {

/**
 * @brief Synchronous CSV logger (not realtime-safe, may cause jitter)
 *
 * This logger writes directly to disk in the control loop.
 * Use AsyncCSVLogger for better realtime performance.
 */
class ControllerCSVLogger : public CSVLoggerInterface {
public:
  ControllerCSVLogger(const std::string& controller_name, rclcpp::Logger logger);
  ~ControllerCSVLogger() override;

  bool initialize(const rclcpp::Time& start_time,
                 const ControllerLogDataInterface& header_generator) override;
  void logData(const ControllerLogDataInterface& data) override;
  void close() override;
  bool isLoggingEnabled() const override { return logging_enabled_ && csv_file_.is_open(); }

private:
  std::string controller_name_;
  rclcpp::Logger logger_;
  std::ofstream csv_file_;
  bool logging_enabled_ = false;
  rclcpp::Time start_time_;
  size_t flush_counter_ = 0;

  static constexpr size_t FLUSH_INTERVAL = 50;
};

} // namespace crisp_controllers