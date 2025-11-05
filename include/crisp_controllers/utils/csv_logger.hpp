#pragma once

#include <fstream>
#include <filesystem>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <pinocchio/spatial/se3.hpp>
#include "crisp_controllers/utils/controller_log_data.hpp"
#include "crisp_controllers/utils/csv_logger_interface.hpp"

namespace crisp_controllers {

class ControllerCSVLogger : public CSVLoggerInterface {
public:
  ControllerCSVLogger(const std::string& controller_name, rclcpp::Logger logger);
  ~ControllerCSVLogger() override;

  bool initialize(size_t num_joints, const rclcpp::Time& start_time) override;
  void logData(const ControllerLogData& data, const rclcpp::Time& current_time) override;
  void close() override;
  bool isLoggingEnabled() const override { return logging_enabled_ && csv_file_.is_open(); }

private:
  void writeHeader(size_t num_joints);
  void writePose(const pinocchio::SE3& pose);
  void writeRPY(const pinocchio::SE3& pose);

  std::string controller_name_;
  rclcpp::Logger logger_;
  std::ofstream csv_file_;
  bool logging_enabled_ = false;
  rclcpp::Time start_time_;
  size_t flush_counter_ = 0;
  size_t num_joints_ = 0;

  static constexpr size_t FLUSH_INTERVAL = 50;
};

} // namespace crisp_controllers