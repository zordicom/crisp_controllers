#pragma once

#include <rclcpp/rclcpp.hpp>
#include "crisp_controllers/utils/controller_log_data.hpp"

namespace crisp_controllers {

/**
 * @brief Abstract interface for CSV logging implementations
 *
 * This interface allows for different logging strategies (synchronous, asynchronous, etc.)
 * while providing a consistent API to the controller.
 */
class CSVLoggerInterface {
public:
  virtual ~CSVLoggerInterface() = default;

  /**
   * @brief Initialize the logger
   * @param num_joints Number of joints in the robot
   * @param start_time Start time for timestamp calculations
   * @return true if initialization was successful
   */
  virtual bool initialize(size_t num_joints, const rclcpp::Time& start_time) = 0;

  /**
   * @brief Log controller data
   * @param data The controller data to log
   * @param current_time Current time for timestamp calculation
   */
  virtual void logData(const ControllerLogData& data, const rclcpp::Time& current_time) = 0;

  /**
   * @brief Close the logger and flush any pending data
   */
  virtual void close() = 0;

  /**
   * @brief Check if logging is currently enabled
   * @return true if logging is active
   */
  virtual bool isLoggingEnabled() const = 0;
};

} // namespace crisp_controllers