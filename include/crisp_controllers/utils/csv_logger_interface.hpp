#pragma once

#include <rclcpp/rclcpp.hpp>
#include "crisp_controllers/utils/controller_log_data_interface.hpp"

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
   * @param start_time Start time for timestamp calculations
   * @param header_generator Function to generate CSV header (first log data)
   * @return true if initialization was successful
   */
  virtual bool initialize(const rclcpp::Time& start_time,
                         const ControllerLogDataInterface& header_generator) = 0;

  /**
   * @brief Log controller data
   * @param data The controller data to log (polymorphic interface)
   */
  virtual void logData(const ControllerLogDataInterface& data) = 0;

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