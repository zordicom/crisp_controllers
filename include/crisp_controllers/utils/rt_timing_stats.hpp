// Copyright (c) 2025, PickNik Robotics
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <time.h>
#include <cmath>
#include <algorithm>

namespace crisp_controllers {

/**
 * @brief POD structure for tracking timing statistics of a single operation
 *
 * Uses online algorithm for variance calculation to avoid storing all samples.
 * RT-safe: no heap allocations, no locks, simple arithmetic only.
 *
 * Statistics tracked:
 * - Average (mean) time
 * - Standard deviation (variance measure)
 * - Maximum observed time
 * - Sample count
 *
 * Formula for standard deviation:
 *   variance = E[X²] - E[X]²
 *           = (sum_of_squares / count) - (average)²
 *   std_dev = sqrt(variance)
 */
struct RTTimingStats {
  // Accumulators for statistics
  long total_us;       ///< Sum of all measurements (microseconds)
  long total_sq_us;    ///< Sum of squared measurements (for variance)
  long max_us;         ///< Maximum observed time (microseconds)
  size_t count;        ///< Number of samples

  // Operation identifier (for logging)
  const char* operation_name;

  /// Default constructor
  RTTimingStats() : total_us(0), total_sq_us(0), max_us(0), count(0), operation_name("") {}

  /**
   * @brief Construct with operation name
   * @param name Human-readable name for this timing statistic
   */
  explicit RTTimingStats(const char* name)
    : total_us(0), total_sq_us(0), max_us(0), count(0), operation_name(name) {}

  /**
   * @brief Add a measurement sample
   * @param duration_us Duration in microseconds
   */
  inline void add_sample(long duration_us) {
    total_us += duration_us;
    total_sq_us += duration_us * duration_us;
    max_us = std::max(max_us, duration_us);
    count++;
  }

  /**
   * @brief Compute average time
   * @return Average time in microseconds, or 0 if no samples
   */
  inline long avg_us() const {
    return (count > 0) ? (total_us / static_cast<long>(count)) : 0;
  }

  /**
   * @brief Compute standard deviation
   * @return Standard deviation in microseconds, or 0 if no samples
   */
  inline long std_us() const {
    if (count == 0) return 0;
    long avg = avg_us();
    long variance = (total_sq_us / static_cast<long>(count)) - (avg * avg);
    return static_cast<long>(std::sqrt(std::max(0L, variance)));
  }

  /**
   * @brief Reset all statistics to zero
   */
  inline void reset() {
    total_us = 0;
    total_sq_us = 0;
    max_us = 0;
    count = 0;
  }
};

/**
 * @brief RT-safe time difference calculation
 *
 * Computes difference between two timespec structs and returns microseconds.
 * Avoids std::chrono to ensure RT-safety.
 *
 * @param start Start time (from clock_gettime)
 * @param end End time (from clock_gettime)
 * @return Duration in microseconds
 */
inline long timespec_diff_us(const struct timespec& start, const struct timespec& end) {
  return ((end.tv_sec - start.tv_sec) * 1000000000L +
          (end.tv_nsec - start.tv_nsec)) / 1000;
}

/**
 * @brief RT-safe timing scope guard (RAII helper)
 *
 * Automatically measures and records timing for a scope block.
 * When the guard is destroyed (end of scope), the elapsed time
 * is automatically recorded to the provided RTTimingStats.
 *
 * Example usage:
 * @code
 *   RTTimingStats my_stats("operation");
 *   {
 *     RTTimingScopeGuard timer(my_stats);
 *     // ... timed operation ...
 *   } // timing recorded automatically here
 * @endcode
 */
struct RTTimingScopeGuard {
  RTTimingStats& stats;           ///< Reference to statistics struct
  struct timespec start_time;     ///< Start time of the scope

  /**
   * @brief Construct and start timing
   * @param s Reference to RTTimingStats to record to
   * @param clock_id Clock to use (default: CLOCK_MONOTONIC)
   */
  RTTimingScopeGuard(RTTimingStats& s, clockid_t clock_id = CLOCK_MONOTONIC)
    : stats(s) {
    clock_gettime(clock_id, &start_time);
  }

  /**
   * @brief Destruct and record timing
   *
   * Automatically called at end of scope. Records elapsed time
   * to the statistics struct provided in constructor.
   */
  ~RTTimingScopeGuard() {
    struct timespec end_time;
    clock_gettime(CLOCK_MONOTONIC, &end_time);
    long duration = timespec_diff_us(start_time, end_time);
    stats.add_sample(duration);
  }
};

} // namespace crisp_controllers
