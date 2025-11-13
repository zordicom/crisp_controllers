#pragma once

#include <string>
#include <sstream>

namespace crisp_controllers {

/**
 * @brief Interface for controller-specific log data
 *
 * Each controller can implement this interface to define exactly what data
 * it wants to log, making it clear which values are actually computed and used.
 */
class ControllerLogDataInterface {
public:
  virtual ~ControllerLogDataInterface() = default;

  /**
   * @brief Get the timestamp for this log entry
   */
  virtual double getTimestamp() const = 0;

  /**
   * @brief Generate CSV header with column names
   * @return CSV header string (without newline)
   */
  virtual std::string generateHeader() const = 0;

  /**
   * @brief Generate CSV data line
   * @return CSV data string (without newline)
   */
  virtual std::string generateLine() const = 0;
};

} // namespace crisp_controllers
