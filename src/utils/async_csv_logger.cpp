#include "crisp_controllers/utils/async_csv_logger.hpp"
#include <iostream>
#include <iomanip>
#include <sched.h>
#include <pinocchio/math/rpy.hpp>

namespace crisp_controllers {

AsyncCSVLogger::AsyncCSVLogger(const std::string& controller_name,
                               rclcpp::Logger logger)
    : controller_name_(controller_name), logger_(logger) {}

AsyncCSVLogger::~AsyncCSVLogger() {
  close();
}

bool AsyncCSVLogger::initialize(size_t num_joints, const rclcpp::Time& start_time) {
  num_joints_ = num_joints;
  start_time_ = start_time;

  // Create log file directory
  std::filesystem::path log_dir = "/tmp/controller_logs";
  std::filesystem::create_directories(log_dir);

  // Generate filename with timestamp
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream filename_stream;
  filename_stream << log_dir.string() << "/" << controller_name_
                  << "_async_log_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
                  << ".csv";

  std::string log_filename = filename_stream.str();
  csv_file_.open(log_filename, std::ios::out);

  if (csv_file_.is_open()) {
    RCLCPP_INFO(logger_, "Async CSV logging enabled, writing to: %s", log_filename.c_str());
    RCLCPP_INFO(logger_, "Max queue size: %zu samples, batch write size: %zu",
                MAX_QUEUE_SIZE, BATCH_WRITE_SIZE);

    writeHeader(num_joints);

    // Start the writer thread
    logging_enabled_ = true;
    shutdown_requested_ = false;
    writer_thread_ = std::thread(&AsyncCSVLogger::writerThread, this);

    // Set thread priority (optional, may require permissions)
    #ifdef __linux__
    struct sched_param param;
    param.sched_priority = WRITER_THREAD_PRIORITY;
    pthread_setschedparam(writer_thread_.native_handle(), SCHED_OTHER, &param);
    #endif

    return true;
  } else {
    RCLCPP_ERROR(logger_, "Failed to open async CSV log file: %s", log_filename.c_str());
    logging_enabled_ = false;
    return false;
  }
}

void AsyncCSVLogger::close() {
  if (logging_enabled_.load()) {
    // Signal shutdown
    shutdown_requested_ = true;
    queue_cv_.notify_all();

    // Wait for writer thread to finish
    if (writer_thread_.joinable()) {
      writer_thread_.join();
    }

    // Close file
    if (csv_file_.is_open()) {
      csv_file_.flush();
      csv_file_.close();

      RCLCPP_INFO(logger_,
                  "Async CSV log file closed. Total samples: %zu, Dropped: %zu (%.2f%%)",
                  total_samples_.load(), dropped_samples_.load(),
                  total_samples_.load() > 0 ?
                    100.0 * dropped_samples_.load() / total_samples_.load() : 0.0);
    }

    logging_enabled_ = false;
  }
}

void AsyncCSVLogger::logData(const ControllerLogData& data, const rclcpp::Time& current_time) {
  if (!logging_enabled_.load()) {
    return;
  }

  total_samples_++;

  // Try to add to queue with minimal locking
  {
    std::unique_lock<std::mutex> lock(queue_mutex_, std::try_to_lock);

    if (!lock.owns_lock()) {
      // Could not acquire lock immediately - skip this sample to avoid blocking RT thread
      dropped_samples_++;
      return;
    }

    // Check queue size
    if (data_queue_.size() >= MAX_QUEUE_SIZE) {
      // Queue is full - drop oldest sample
      data_queue_.pop_front();
      dropped_samples_++;
    }

    // Add new data
    data_queue_.push_back(data);
    queue_size_ = data_queue_.size();
  }

  // Notify writer thread
  queue_cv_.notify_one();
}

void AsyncCSVLogger::writerThread() {
  RCLCPP_INFO(logger_, "CSV writer thread started");

  std::vector<ControllerLogData> batch_buffer;
  batch_buffer.reserve(BATCH_WRITE_SIZE);

  while (!shutdown_requested_.load()) {
    // Wait for data or shutdown signal
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this] {
        return !data_queue_.empty() || shutdown_requested_.load();
      });

      // Collect a batch of data
      while (!data_queue_.empty() && batch_buffer.size() < BATCH_WRITE_SIZE) {
        batch_buffer.push_back(std::move(data_queue_.front()));
        data_queue_.pop_front();
      }

      queue_size_ = data_queue_.size();
    }

    // Write batch to file (outside of lock)
    for (const auto& data : batch_buffer) {
      processLogData(data);
    }

    batch_buffer.clear();
  }

  // Process any remaining data before shutdown
  {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    while (!data_queue_.empty()) {
      processLogData(data_queue_.front());
      data_queue_.pop_front();
    }
  }

  RCLCPP_INFO(logger_, "CSV writer thread finished");
}

void AsyncCSVLogger::processLogData(const ControllerLogData& data) {
  if (!csv_file_.is_open()) {
    return;
  }

  // Write timestamp
  csv_file_ << data.timestamp;

  // Write task space forces
  for (int i = 0; i < 6; ++i) {
    csv_file_ << "," << data.task_force_P[i];
  }
  for (int i = 0; i < 6; ++i) {
    csv_file_ << "," << data.task_force_D[i];
  }
  for (int i = 0; i < 6; ++i) {
    csv_file_ << "," << data.task_force_total[i];
  }

  // Write torque components
  for (int i = 0; i < data.tau_task.size(); ++i) {
    csv_file_ << "," << data.tau_task[i]
              << "," << data.tau_nullspace[i]
              << "," << data.tau_joint_limits[i]
              << "," << data.tau_friction[i]
              << "," << data.tau_coriolis[i]
              << "," << data.tau_gravity[i]
              << "," << data.tau_wrench[i]
              << "," << data.tau_total[i];
  }

  // Write error metrics
  for (int i = 0; i < 6; ++i) {
    csv_file_ << "," << data.error[i];
  }
  csv_file_ << "," << data.error_rot_magnitude << "," << data.error_pos_magnitude;

  // Write poses
  writePose(data.current_pose);
  writeRPY(data.current_pose);
  writePose(data.target_pose);
  writeRPY(data.target_pose);

  // Write stiffness and damping
  for (int i = 0; i < 6; ++i) {
    csv_file_ << "," << data.stiffness_diag[i];
  }
  for (int i = 0; i < 6; ++i) {
    csv_file_ << "," << data.damping_diag[i];
  }

  // Write joint states
  for (int i = 0; i < data.q_raw.size(); ++i) {
    csv_file_ << "," << data.q_raw[i];
  }
  for (int i = 0; i < data.q_filtered.size(); ++i) {
    csv_file_ << "," << data.q_filtered[i];
  }
  for (int i = 0; i < data.dq_raw.size(); ++i) {
    csv_file_ << "," << data.dq_raw[i];
  }
  for (int i = 0; i < data.dq_filtered.size(); ++i) {
    csv_file_ << "," << data.dq_filtered[i];
  }
  for (int i = 0; i < data.q_goal.size(); ++i) {
    csv_file_ << "," << data.q_goal[i];
  }
  for (int i = 0; i < data.dq_goal.size(); ++i) {
    csv_file_ << "," << data.dq_goal[i];
  }

  // Write filter parameters
  csv_file_ << "," << data.filter_q << "," << data.filter_dq << "," << data.filter_output_torque;

  // Use '\n' for efficiency (no flush)
  csv_file_ << '\n';

  // Periodic flush for data safety (every 100 samples)
  static size_t flush_counter = 0;
  if (++flush_counter >= 100) {
    csv_file_.flush();
    flush_counter = 0;
  }
}

void AsyncCSVLogger::writeHeader(size_t num_joints) {
  // Write CSV header
  csv_file_ << "timestamp";

  // Task space forces header
  csv_file_ << ",task_force_P_x,task_force_P_y,task_force_P_z,task_"
               "force_P_rx,task_force_P_ry,task_force_P_rz";
  csv_file_ << ",task_force_D_x,task_force_D_y,task_force_D_z,task_"
               "force_D_rx,task_force_D_ry,task_force_D_rz";
  csv_file_ << ",task_force_total_x,task_force_total_y,task_force_total_z,"
               "task_force_total_rx,task_force_total_ry,task_force_total_rz";

  // Torque components header (per joint)
  for (size_t i = 0; i < num_joints; ++i) {
    csv_file_ << ",tau_task_" << i << ",tau_nullspace_" << i
              << ",tau_joint_limits_" << i << ",tau_friction_" << i
              << ",tau_coriolis_" << i << ",tau_gravity_" << i
              << ",tau_wrench_" << i << ",tau_total_" << i;
  }

  // Error metrics header
  csv_file_ << ",error_x,error_y,error_z,error_rx,error_ry,error_rz";
  csv_file_ << ",error_rot_magnitude,error_pos_magnitude";

  // Pose headers
  csv_file_ << ",current_x,current_y,current_z,current_qw,current_qx,"
               "current_qy,current_qz";
  csv_file_ << ",current_roll,current_pitch,current_yaw";
  csv_file_ << ",target_x,target_y,target_z,target_qw,target_qx,target_"
               "qy,target_qz";
  csv_file_ << ",target_roll,target_pitch,target_yaw";

  // Stiffness and damping headers
  csv_file_ << ",k_pos_x,k_pos_y,k_pos_z,k_rot_x,k_rot_y,k_rot_z";
  csv_file_ << ",d_pos_x,d_pos_y,d_pos_z,d_rot_x,d_rot_y,d_rot_z";

  // Joint states headers
  for (size_t i = 0; i < num_joints; ++i) {
    csv_file_ << ",q_raw_" << i;
  }
  for (size_t i = 0; i < num_joints; ++i) {
    csv_file_ << ",q_filtered_" << i;
  }
  for (size_t i = 0; i < num_joints; ++i) {
    csv_file_ << ",dq_raw_" << i;
  }
  for (size_t i = 0; i < num_joints; ++i) {
    csv_file_ << ",dq_filtered_" << i;
  }
  for (size_t i = 0; i < num_joints; ++i) {
    csv_file_ << ",q_goal_" << i;
  }
  for (size_t i = 0; i < num_joints; ++i) {
    csv_file_ << ",dq_goal_" << i;
  }

  // Filter parameters header
  csv_file_ << ",filter_q,filter_dq,filter_output_torque";

  csv_file_ << std::endl;
}

void AsyncCSVLogger::writePose(const pinocchio::SE3& pose) {
  const auto& trans = pose.translation();
  const auto& quat = Eigen::Quaterniond(pose.rotation());

  csv_file_ << "," << trans[0] << "," << trans[1] << "," << trans[2];
  csv_file_ << "," << quat.w() << "," << quat.x() << "," << quat.y() << "," << quat.z();
}

void AsyncCSVLogger::writeRPY(const pinocchio::SE3& pose) {
  auto rpy = pinocchio::rpy::matrixToRpy(pose.rotation());
  csv_file_ << "," << rpy[0] << "," << rpy[1] << "," << rpy[2];
}

} // namespace crisp_controllers