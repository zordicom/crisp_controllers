#pragma once

#include <stdexcept>

/**
  * @brief Compute the exponential moving average of a value
  * @param output Previous output value
  * @param current Current value measured to be filtered
   * @param alpha Smoothing factor (0 < alpha < 1). The closer to 1, the more it
  * smooths the value
  * @return returns the filtered value
  */
template <typename T>
  inline T exponential_moving_average(T output, T current, double alpha)
  {
  if (alpha <= 0.0 || alpha >= 1.0) {
      throw std::invalid_argument("alpha must be in (0, 1)");
  }

    return (1.0 - alpha) * current + alpha * output;
  }

