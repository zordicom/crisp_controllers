#include "crisp_controllers/utils/csv_logger_interface.hpp"
#include "crisp_controllers/utils/async_csv_logger.hpp"
#include "crisp_controllers/utils/csv_logger.hpp"
#include "crisp_controllers/utils/fiters.hpp"
#include "crisp_controllers/utils/torque_rate_saturation.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <controller_interface/controller_interface_base.hpp>
#include <crisp_controllers/cartesian_controller.hpp>
#include <crisp_controllers/pch.hpp>
#include <crisp_controllers/utils/friction_model.hpp>
#include <crisp_controllers/utils/joint_limits.hpp>
#include <crisp_controllers/utils/pseudo_inverse.hpp>

#include "pinocchio/algorithm/model.hpp"
#include <cstddef>
#include <fmt/format.h>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hxx>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/fwd.hpp>
#include <rclcpp/logging.hpp>

namespace crisp_controllers {

controller_interface::InterfaceConfiguration
CartesianController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : params_.joints) {
    config.names.push_back(joint_name + "/position");
  }
  for (const auto &joint_name : params_.joints) {
    config.names.push_back(joint_name + "/velocity");
  }
  for (const auto &joint_name : params_.joints) {
    config.names.push_back(joint_name + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
CartesianController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto &joint_name : params_.joints) {
    config.names.push_back(joint_name + "/position");
  }
  for (const auto &joint_name : params_.joints) {
    config.names.push_back(joint_name + "/velocity");
  }
  for (const auto &joint_name : params_.joints) {
    config.names.push_back(joint_name + "/effort");
  }
  return config;
}

controller_interface::return_type
CartesianController::update(const rclcpp::Time &time,
                            const rclcpp::Duration & /*period*/) {

  // Start timing the control loop
  auto loop_start_time = get_node()->get_clock()->now();

  size_t num_joints = params_.joints.size();

  // Log first update to compare with on_activate values
  static bool first_update = true;
  if (first_update) {
    RCLCPP_INFO(get_node()->get_logger(),
                "First update() call - checking joint positions...");
  }

  for (size_t i = 0; i < num_joints; i++) {
    // Use cached joint IDs and models (moved from control loop to
    // configuration)
    const auto &joint = *joint_models_[i];

    // Store raw values from hardware for logging
    q_raw[i] = state_interfaces_[i].get_value();
    dq_raw[i] = state_interfaces_[num_joints + i].get_value();

    // Apply exponential moving average filtering
    q[i] = exponential_moving_average(q[i], q_raw[i], params_.filter.q);
    dq[i] = exponential_moving_average(dq[i], dq_raw[i], params_.filter.dq);

    if (continous_joint_types.count(
            joint.shortname())) { // Then we are handling a continous
                                  // joint that is SO(2)
      q_pin[joint.idx_q()] = std::cos(q[i]);
      q_pin[joint.idx_q() + 1] = std::sin(q[i]);
    } else { // simple revolute joint case
      q_pin[joint.idx_q()] = q[i];
    }

    // Log velocity filtering for first joint to verify it's working
    if (i == 0) {
      // Throttle logging to every LOG_CYCLE_INTERVAL cycles (~1Hz at 100Hz
      // update rate)
      static int vel_log_counter = 0;
      if (vel_log_counter++ % LOG_CYCLE_INTERVAL == 0) {
        RCLCPP_INFO(get_node()->get_logger(),
                    "Joint0: q_raw=%.4f q_filt=%.4f dq_raw=%.4f dq_filt=%.4f "
                    "alpha_dq=%.3f",
                    q_raw[i], q[i], dq_raw[i], dq[i], params_.filter.dq);
      }
    }
  }

  if (new_target_pose_) {
    parse_target_pose_();
    new_target_pose_ = false;
  }
  if (new_target_joint_) {
    parse_target_joint_();
    new_target_joint_ = false;
  }
  if (new_target_wrench_) {
    parse_target_wrench_();
    new_target_wrench_ = false;
  }

  pinocchio::forwardKinematics(model_, data_, q_pin, dq);
  pinocchio::updateFramePlacements(model_, data_);

  // Apply filtering to target pose (already in world frame from parse_target_pose_)
  pinocchio::SE3 new_target_pose =
      pinocchio::SE3(target_orientation_.toRotationMatrix(), target_position_);

  target_pose_ = pinocchio::exp6(exponential_moving_average(
      pinocchio::log6(target_pose_), pinocchio::log6(new_target_pose),
      params_.filter.target_pose));

  // Get end-effector pose in world frame (as Pinocchio provides it)
  ee_pose_world_ = data_.oMf[ee_frame_id_];

  // Log first update to verify state interface data
  if (first_update) {
    RCLCPP_INFO(get_node()->get_logger(),
                "First update: Joint positions: [%.3f, %.3f, %.3f, %.3f, %.3f, "
                "%.3f, %.3f]",
                q[0], q[1], q[2], q[3], q[4], q[5], q[6]);
    Eigen::Vector3d current_pos = ee_pose_world_.translation();
    Eigen::Quaterniond current_quat(ee_pose_world_.rotation());
    RCLCPP_INFO(
        get_node()->get_logger(),
        "First update: Current end-effector position (world): [%.3f, %.3f, %.3f]",
        current_pos.x(), current_pos.y(), current_pos.z());
    RCLCPP_INFO(
        get_node()->get_logger(),
        "First update: Target end-effector position (world): [%.3f, %.3f, %.3f]",
        target_position_.x(), target_position_.y(), target_position_.z());
    first_update = false;
  }

  // Compute error in WORLD frame (both target_pose_ and ee_pose_world_ are in world frame)
  // We consider translation and rotation separately to avoid unnatural screw motions
  error.head(3) = target_pose_.translation() - ee_pose_world_.translation();
  error.tail(3) = pinocchio::log3(target_pose_.rotation() *
                                  ee_pose_world_.rotation().transpose());

  if (params_.limit_error) {
    max_delta_ << params_.task.error_clip.x, params_.task.error_clip.y,
        params_.task.error_clip.z, params_.task.error_clip.rx,
        params_.task.error_clip.ry, params_.task.error_clip.rz;
    error = error.cwiseMax(-max_delta_).cwiseMin(max_delta_);
  }

  // Log error every LOG_CYCLE_INTERVAL cycles (~1Hz at 100Hz update rate)
  static int log_counter = 0;
  if (log_counter++ % LOG_CYCLE_INTERVAL == 0) {
    RCLCPP_INFO(
        get_node()->get_logger(),
        "Cartesian error: pos=[%.4f, %.4f, %.4f]m, ori=[%.4f, %.4f, %.4f]rad",
        error[0], error[1], error[2], error[3], error[4], error[5]);
  }

  J.setZero();
  // Always use WORLD frame as reference - this is the most straightforward approach
  pinocchio::computeFrameJacobian(model_, data_, q_pin, ee_frame_id_,
                                  pinocchio::ReferenceFrame::WORLD, J);

  // No transformation needed - we'll work directly in the world frame
  // The base_frame is only used for interpreting target poses, not for Jacobian computation

  J_pinv_ = pseudo_inverse(J, params_.nullspace.regularization);

  // Compute Minverse once if needed by either dynamic nullspace or operational
  // space
  bool minverse_computed = false; // Currently unused

  if (params_.nullspace.projector_type == "dynamic" ||
      params_.use_operational_space) {
    pinocchio::computeMinverse(model_, data_, q_pin);
    minverse_computed = true;

    // Compute operational space mass matrix (used by both dynamic nullspace and
    // OSC)
    Mx_inv_ = J * data_.Minv * J.transpose();
    Mx_ = pseudo_inverse(Mx_inv_);
  }

  if (params_.nullspace.projector_type == "dynamic") {
    // Use already computed Minv and Mx
    J_bar_ = data_.Minv * J.transpose() * Mx_;
    nullspace_projection = Id_nv_ - J.transpose() * J_bar_.transpose();
  } else if (params_.nullspace.projector_type == "kinematic") {
    nullspace_projection = Id_nv_ - J_pinv_ * J;
  } else if (params_.nullspace.projector_type == "none") {
    nullspace_projection = Id_nv_;
  } else {
    RCLCPP_ERROR_STREAM_ONCE(get_node()->get_logger(),
                             "Unknown nullspace projector type: "
                                 << params_.nullspace.projector_type);
    return controller_interface::return_type::ERROR;
  }

  // Compute task space forces separately for logging
  task_force_P_ = stiffness * error;        // Proportional term
  task_velocity_ = J * dq;                  // Task space velocity
  task_force_D_ = damping * task_velocity_; // Damping term

  // Log damping force computation to verify filtering effect
  static int damping_log_counter = 0;
  if (damping_log_counter++ % LOG_CYCLE_INTERVAL == 0) {
    RCLCPP_INFO(get_node()->get_logger(),
                "Damping: task_vel_x=%.4f, d_pos=%.2f, force_D_x=%.4f (P=%.4f)",
                task_velocity_[0], damping(0, 0), task_force_D_[0],
                task_force_P_[0]);
  }

  if (params_.use_operational_space) {
    // Minv and Mx already computed above if needed
    task_force_total_ = Mx_ * (task_force_P_ - task_force_D_);
    tau_task = J.transpose() * task_force_total_;
  } else {
    task_force_total_ = task_force_P_ - task_force_D_;
    tau_task = J.transpose() * task_force_total_;
  }

  // Store task space forces for CSV logging later in log_debug_info
  // (These are temporary variables that will be accessed in log_debug_info)

  if (model_.nq != model_.nv) {
    // TODO: Then we have some continouts joints, not being handled for now
    tau_joint_limits.setZero();
  } else if (params_.joint_limit_avoidance.enable) {
    tau_joint_limits = get_joint_limit_torque(
        q, model_.lowerPositionLimit, model_.upperPositionLimit,
        params_.joint_limit_avoidance.safe_range,
        params_.joint_limit_avoidance.max_torque);
  } else {
    tau_joint_limits.setZero();
  }

  tau_secondary =
      nullspace_stiffness * (q_ref - q) + nullspace_damping * (dq_ref - dq);

  tau_nullspace = nullspace_projection * tau_secondary;
  tau_nullspace = tau_nullspace.cwiseMin(params_.nullspace.max_tau)
                      .cwiseMax(-params_.nullspace.max_tau);

  if (params_.use_friction) {
    tau_friction = get_friction(dq, fp1, fp2, fp3);
  } else {
    tau_friction.setZero();
  }

  if (params_.use_coriolis_compensation) {
    // TODO: This might be redundant - computeAllTerms already computes C*dq+g
    // in data_.nle Could potentially use: tau_coriolis = data_.nle -
    // gravity_terms But needs careful testing to ensure correctness
    pinocchio::computeAllTerms(model_, data_, q_pin, dq);
    tau_coriolis =
        pinocchio::computeCoriolisMatrix(model_, data_, q_pin, dq) * dq;
  } else {
    tau_coriolis.setZero();
  }

  if (params_.use_gravity_compensation) {
    tau_gravity = params_.gravity_scale *
                  pinocchio::computeGeneralizedGravity(model_, data_, q_pin);
  } else {
    tau_gravity.setZero();
  }

  tau_wrench = J.transpose() * target_wrench_;

  // Gravity-only mode: only apply gravity compensation (safe for gain tuning)
  if (params_.gravity_only_mode) {
    tau_d = tau_gravity;
  } else {
    tau_d = tau_task + tau_nullspace + tau_friction + tau_coriolis +
            tau_gravity + tau_joint_limits + tau_wrench;
  }

  if (params_.limit_torques) {
    tau_d = saturateTorqueRate(tau_d, tau_previous, params_.max_delta_tau);
  }

  // Apply absolute torque limits from URDF/Pinocchio model
  // This ensures we never exceed motor torque capabilities
  // Using pre-calculated limits with safety factor for efficiency
  tau_d_unclamped_ = tau_d;

  // Vectorized clamping: tau_d = min(max(tau_d, -limits), limits)
  tau_d = tau_d.cwiseMin(tau_limits).cwiseMax(-tau_limits);

  // Log commanded torques every LOG_CYCLE_INTERVAL cycles (~1Hz at 100Hz update
  // rate)
  static int torque_log_counter = 0;
  if (torque_log_counter++ % LOG_CYCLE_INTERVAL == 0) {
    RCLCPP_INFO(get_node()->get_logger(),
                "Commanded torques: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f] "
                "Nm (stop_commands=%s, gravity_only=%s)",
                tau_d[0], tau_d[1], tau_d[2], tau_d[3], tau_d[4], tau_d[5],
                tau_d[6], params_.stop_commands ? "TRUE" : "FALSE",
                params_.gravity_only_mode ? "TRUE" : "FALSE");

    // Log if any torques were saturated
    bool saturated = false;
    for (size_t i = 0; i < num_joints; ++i) {
      if (std::abs(tau_d_unclamped_[i] - tau_d[i]) > 0.01) {
        saturated = true;
        break;
      }
    }
    if (saturated) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Torque saturation active! Unclamped: [%.3f, %.3f, %.3f, "
                  "%.3f, %.3f, %.3f, %.3f] Nm",
                  tau_d_unclamped_[0], tau_d_unclamped_[1], tau_d_unclamped_[2],
                  tau_d_unclamped_[3], tau_d_unclamped_[4], tau_d_unclamped_[5],
                  tau_d_unclamped_[6]);
    }
  }

  if (not params_.stop_commands) {
    for (size_t i = 0; i < num_joints; ++i) {
      command_interfaces_[i].set_value(0.0);
      command_interfaces_[num_joints + i].set_value(0.0);
      command_interfaces_[2 * num_joints + i].set_value(tau_d[i]);
    }
  } else {
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "stop_commands is TRUE - not sending torques to hardware!");
  }

  tau_previous = tau_d;

  // Only refresh parameters periodically to avoid mutex locks every cycle
  double elapsed_ms = (time - last_param_refresh_time_).seconds() * 1000.0;
  if (elapsed_ms >= PARAM_REFRESH_INTERVAL_MS) {
    params_listener_->refresh_dynamic_parameters();
    params_ = params_listener_->get_params();
    setStiffnessAndDamping();
    last_param_refresh_time_ = time;
  }

  log_debug_info(time, loop_start_time);

  return controller_interface::return_type::OK;
}

CallbackReturn CartesianController::on_init() {
  params_listener_ =
      std::make_shared<cartesian_impedance_controller::ParamListener>(
          get_node());
  params_listener_->refresh_dynamic_parameters();
  params_ = params_listener_->get_params();

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  RCLCPP_INFO(get_node()->get_logger(),
              "Starting CartesianController configuration...");

  // Create a separate node and executor to avoid deadlock with
  // controller_manager's executor
  auto param_node = std::make_shared<rclcpp::Node>(
      "cartesian_controller_param_client",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          false));

  auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
      param_node, "robot_state_publisher");

  RCLCPP_INFO(get_node()->get_logger(),
              "Waiting for robot_state_publisher service...");
  parameters_client->wait_for_service();
  RCLCPP_INFO(get_node()->get_logger(), "robot_state_publisher service found!");

  RCLCPP_INFO(get_node()->get_logger(),
              "Getting robot_description parameter...");
  auto future = parameters_client->get_parameters({"robot_description"});

  // Spin the separate node to process the async response
  while (rclcpp::ok() && future.wait_for(std::chrono::milliseconds(100)) !=
                             std::future_status::ready) {
    rclcpp::spin_some(param_node);
  }

  auto result = future.get();
  RCLCPP_INFO(get_node()->get_logger(), "Got robot_description parameter!");

  std::string robot_description_;
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to get robot_description parameter.");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(),
              "Building Pinocchio model from URDF...");
  pinocchio::Model raw_model_;

  // Build model with explicit root joint (fixed to world)
  // This doesn't change the reference frame but makes the root explicit
  pinocchio::urdf::buildModelFromXML(robot_description_, raw_model_);
  RCLCPP_INFO(get_node()->get_logger(), "Pinocchio model built successfully!");

  RCLCPP_INFO(get_node()->get_logger(), "Checking available joints in model:");
  for (int joint_id = 0; joint_id < raw_model_.njoints; joint_id++) {
    RCLCPP_INFO_STREAM(get_node()->get_logger(),
                       "Joint " << joint_id << " with name "
                                << raw_model_.names[joint_id] << " is of type "
                                << raw_model_.joints[joint_id].shortname());
  }

  // First we check that the passed joints exist in the kineatic tree
  for (auto &joint : params_.joints) {
    if (not raw_model_.existJointName(joint)) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                          "Failed to configure because "
                              << joint
                              << " is not part of the kinematic tree but it "
                                 "has been passed in the parameters.");
      return CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(get_node()->get_logger(),
              "All joints passed in the parameters exist in the kinematic tree "
              "of the URDF.");
  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "Removing the rest of the joints that are not used: ");
  // Now we fix all joints that are not referenced in the tree
  std::vector<pinocchio::JointIndex> list_of_joints_to_lock_by_id;
  for (auto &joint : raw_model_.names) {
    if (std::find(params_.joints.begin(), params_.joints.end(), joint) ==
            params_.joints.end() and
        joint != "universe") {
      RCLCPP_INFO_STREAM(
          get_node()->get_logger(),
          "Joint " << joint << " is not used, removing it from the model.");
      list_of_joints_to_lock_by_id.push_back(raw_model_.getJointId(joint));
    }
  }

  RCLCPP_INFO(get_node()->get_logger(), "Building reduced Pinocchio model...");
  Eigen::VectorXd q_locked = Eigen::VectorXd::Zero(raw_model_.nq);
  model_ = pinocchio::buildReducedModel(raw_model_,
                                        list_of_joints_to_lock_by_id, q_locked);
  data_ = pinocchio::Data(model_);
  RCLCPP_INFO(get_node()->get_logger(), "Reduced model built successfully!");

  RCLCPP_INFO(get_node()->get_logger(), "Validating joint types...");
  for (int joint_id = 0; joint_id < model_.njoints; joint_id++) {
    if (model_.names[joint_id] == "universe") {
      continue;
    }
    if (not allowed_joint_types.count(model_.joints[joint_id].shortname())) {
      RCLCPP_ERROR_STREAM(
          get_node()->get_logger(),
          "Joint type "
              << model_.joints[joint_id].shortname() << " is unsupported ("
              << model_.names[joint_id]
              << "), only revolute/continous like joints can be used.");
      return CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Looking for end effector frame: " << params_.end_effector_frame);

  if (!model_.existFrame(params_.end_effector_frame)) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        "End effector frame '" << params_.end_effector_frame
                                               << "' not found in model!");
    RCLCPP_ERROR(get_node()->get_logger(), "Available frames:");
    for (size_t i = 0; i < model_.frames.size(); i++) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                          "  - " << model_.frames[i].name);
    }
    return CallbackReturn::ERROR;
  }

  ee_frame_id_ = model_.getFrameId(params_.end_effector_frame);
  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "Found end effector frame with ID: " << ee_frame_id_);

  // Base frame is required for proper operation
  if (params_.base_frame.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "base_frame parameter is required but not specified!");
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "Please set the base_frame parameter in the controller configuration.");
    return CallbackReturn::ERROR;
  }

  // Check if base frame exists in the model
  if (!model_.existFrame(params_.base_frame)) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        "Base frame '" << params_.base_frame
                                       << "' not found in model!");
    RCLCPP_ERROR(get_node()->get_logger(), "Available frames:");
    for (size_t i = 0; i < model_.frames.size(); i++) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                          "  - " << model_.frames[i].name);
    }
    return CallbackReturn::ERROR;
  }

  base_frame_id_ = model_.getFrameId(params_.base_frame);
  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "Found base frame '" << params_.base_frame
                                          << "' with ID: " << base_frame_id_);
  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "Expecting target poses in frame: " << params_.base_frame);

  // Validate that base frame doesn't go through any actuated joints
  // This ensures base_frame_pose_world_ remains constant
  const auto& base_frame = model_.frames[base_frame_id_];
  if (base_frame.parentJoint != 0) { // If not attached to universe/root
    // Check if any joint in the kinematic chain to root is actuated
    pinocchio::JointIndex current_joint = base_frame.parentJoint;
    while (current_joint != 0) {
      const auto& joint_name = model_.names[current_joint];
      if (std::find(params_.joints.begin(), params_.joints.end(), joint_name) !=
          params_.joints.end()) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Base frame '%s' goes through actuated joint '%s'!",
                     params_.base_frame.c_str(), joint_name.c_str());
        RCLCPP_ERROR(get_node()->get_logger(),
                     "The base frame must be fixed relative to the world frame. "
                     "Please choose a frame that doesn't move with any actuated joints.");
        return CallbackReturn::ERROR;
      }
      // Move up the kinematic chain
      current_joint = model_.parents[current_joint];
    }
  }

  RCLCPP_INFO(get_node()->get_logger(),
              "Base frame '%s' validated - it's fixed relative to world frame",
              params_.base_frame.c_str());

  q = Eigen::VectorXd::Zero(model_.nv);
  q_raw = Eigen::VectorXd::Zero(model_.nv);
  q_pin = Eigen::VectorXd::Zero(model_.nq);
  dq = Eigen::VectorXd::Zero(model_.nv);
  dq_raw = Eigen::VectorXd::Zero(model_.nv);
  q_ref = Eigen::VectorXd::Zero(model_.nv);
  dq_ref = Eigen::VectorXd::Zero(model_.nv);
  tau_previous = Eigen::VectorXd::Zero(model_.nv);
  J = Eigen::MatrixXd::Zero(6, model_.nv);

  // Pre-allocate matrices for control loop (avoiding per-cycle allocations)
  J_pinv_ = Eigen::MatrixXd::Zero(model_.nv, 6);
  Id_nv_ = Eigen::MatrixXd::Identity(model_.nv, model_.nv);
  J_bar_ = Eigen::MatrixXd::Zero(model_.nv, 6);
  tau_d_unclamped_ = Eigen::VectorXd::Zero(model_.nv);
  // Note: task_velocity_, Mx_inv_, Mx_ are fixed-size and don't need initialization

  // Pre-calculate torque limits with safety factor
  tau_limits = model_.effortLimit * params_.torque_safety_factor;
  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "Torque limits initialized (safety factor="
                         << params_.torque_safety_factor
                         << "): " << tau_limits.transpose());

  // Map the friction parameters to Eigen vectors
  fp1 = Eigen::Map<Eigen::VectorXd>(params_.friction.fp1.data(), model_.nv);
  fp2 = Eigen::Map<Eigen::VectorXd>(params_.friction.fp2.data(), model_.nv);
  fp3 = Eigen::Map<Eigen::VectorXd>(params_.friction.fp3.data(), model_.nv);

  nullspace_stiffness = Eigen::MatrixXd::Zero(model_.nv, model_.nv);
  nullspace_damping = Eigen::MatrixXd::Zero(model_.nv, model_.nv);

  // Cache joint IDs and models to avoid lookups in the control loop
  size_t num_joints = params_.joints.size();
  joint_ids_.resize(num_joints);
  joint_models_.resize(num_joints);

  for (size_t i = 0; i < num_joints; i++) {
    const auto &joint_name = params_.joints[i];
    joint_ids_[i] = model_.getJointId(joint_name);
    joint_models_[i] = &model_.joints[joint_ids_[i]];

    // Verify the joint exists
    if (joint_ids_[i] >= model_.joints.size()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Joint '%s' not found in model!",
                   joint_name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(get_node()->get_logger(), "Cached %zu joint IDs for control loop",
              num_joints);

  setStiffnessAndDamping();

  new_target_pose_ = false;
  new_target_joint_ = false;
  new_target_wrench_ = false;

  multiple_publishers_detected_ = false;
  max_allowed_publishers_ = 1;

  auto target_pose_callback =
      [this](
          const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg) -> void {
    if (!check_topic_publisher_count("target_pose")) {
      RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(),
          DEBUG_LOG_THROTTLE_MS,
          "Ignoring target_pose message due to multiple publishers detected!");
      return;
    }

    // Validate frame_id if base_frame is specified
    if (!params_.base_frame.empty() &&
        msg->header.frame_id != params_.base_frame) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Rejecting target pose: frame_id '%s' does not match "
                   "expected base_frame '%s'",
                   msg->header.frame_id.c_str(), params_.base_frame.c_str());
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "Please ensure the target pose is published in the '%s' frame",
          params_.base_frame.c_str());
      return;
    }

    RCLCPP_INFO(get_node()->get_logger(),
                "Received target pose in %s frame: pos=[%.3f, %.3f, %.3f], "
                "ori=[%.3f, %.3f, %.3f, %.3f]",
                msg->header.frame_id.c_str(), msg->pose.position.x,
                msg->pose.position.y, msg->pose.position.z,
                msg->pose.orientation.w, msg->pose.orientation.x,
                msg->pose.orientation.y, msg->pose.orientation.z);
    RCLCPP_DEBUG(get_node()->get_logger(),
                 "Target pose will be transformed from %s to world frame",
                 params_.base_frame.c_str());
    target_pose_buffer_.writeFromNonRT(msg);
    new_target_pose_ = true;
  };

  auto target_joint_callback =
      [this](const std::shared_ptr<sensor_msgs::msg::JointState> msg) -> void {
    if (!check_topic_publisher_count("target_joint")) {
      RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(),
          DEBUG_LOG_THROTTLE_MS,
          "Ignoring target_joint message due to multiple publishers detected!");
      return;
    }
    target_joint_buffer_.writeFromNonRT(msg);
    new_target_joint_ = true;
  };

  auto target_wrench_callback =
      [this](const std::shared_ptr<geometry_msgs::msg::WrenchStamped> msg)
      -> void {
    if (!check_topic_publisher_count("target_wrench")) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                           1000,
                           "Ignoring target_wrench message due to multiple "
                           "publishers detected!");
      return;
    }
    target_wrench_buffer_.writeFromNonRT(msg);
    new_target_wrench_ = true;
  };

  // Create node-private topics using explicit namespacing
  // This ensures each controller instance has its own topic namespace
  std::string node_name = get_node()->get_name();
  std::string pose_topic = node_name + "/target_pose";
  std::string joint_topic = node_name + "/target_joint";
  std::string wrench_topic = node_name + "/target_wrench";

  RCLCPP_INFO(get_node()->get_logger(), "Subscribing to topics:");
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "  Pose: " << pose_topic);
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "  Joint: " << joint_topic);
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "  Wrench: " << wrench_topic);

  pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic, rclcpp::QoS(1), target_pose_callback);

  joint_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      joint_topic, rclcpp::QoS(1), target_joint_callback);

  wrench_sub_ =
      get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
          wrench_topic, rclcpp::QoS(1), target_wrench_callback);

  // Initialize all control vectors with appropriate dimensions
  tau_task = Eigen::VectorXd::Zero(model_.nv);
  tau_joint_limits = Eigen::VectorXd::Zero(model_.nv);
  tau_secondary = Eigen::VectorXd::Zero(model_.nv);
  tau_nullspace = Eigen::VectorXd::Zero(model_.nv);
  tau_friction = Eigen::VectorXd::Zero(model_.nv);
  tau_coriolis = Eigen::VectorXd::Zero(model_.nv);
  tau_gravity = Eigen::VectorXd::Zero(model_.nv);
  tau_wrench = Eigen::VectorXd::Zero(model_.nv);
  tau_d = Eigen::VectorXd::Zero(model_.nv);

  // Initialize target state vectors
  target_position_ = Eigen::Vector3d::Zero();
  target_orientation_ = Eigen::Quaterniond::Identity();
  target_wrench_ = Eigen::VectorXd::Zero(6);
  target_pose_ = pinocchio::SE3::Identity();

  // Initialize error vector
  error = Eigen::VectorXd::Zero(6);
  max_delta_ = Eigen::VectorXd::Zero(6);

  // Initialize nullspace projection matrix
  nullspace_projection = Eigen::MatrixXd::Identity(model_.nv, model_.nv);

  // Initialize parameter refresh tracking
  last_param_refresh_time_ = get_node()->now();

  RCLCPP_INFO(get_node()->get_logger(),
              "State interfaces and control vectors initialized.");

  RCLCPP_INFO(get_node()->get_logger(),
              "CartesianController configuration completed successfully!");

  return CallbackReturn::SUCCESS;
}

void CartesianController::setStiffnessAndDamping() {

  // Update torque limits if safety factor changed
  tau_limits = model_.effortLimit * params_.torque_safety_factor;

  stiffness.setZero();
  stiffness.diagonal() << params_.task.k_pos_x, params_.task.k_pos_y,
      params_.task.k_pos_z, params_.task.k_rot_x, params_.task.k_rot_y,
      params_.task.k_rot_z;

  damping.setZero();
  // For each axis, use explicit damping if > 0, otherwise compute from
  // stiffness
  damping.diagonal() << (params_.task.d_pos_x > 0
                             ? params_.task.d_pos_x
                             : 2.0 * std::sqrt(params_.task.k_pos_x)),
      (params_.task.d_pos_y > 0 ? params_.task.d_pos_y
                                : 2.0 * std::sqrt(params_.task.k_pos_y)),
      (params_.task.d_pos_z > 0 ? params_.task.d_pos_z
                                : 2.0 * std::sqrt(params_.task.k_pos_z)),
      (params_.task.d_rot_x > 0 ? params_.task.d_rot_x
                                : 2.0 * std::sqrt(params_.task.k_rot_x)),
      (params_.task.d_rot_y > 0 ? params_.task.d_rot_y
                                : 2.0 * std::sqrt(params_.task.k_rot_y)),
      (params_.task.d_rot_z > 0 ? params_.task.d_rot_z
                                : 2.0 * std::sqrt(params_.task.k_rot_z));

  nullspace_stiffness.setZero();
  nullspace_damping.setZero();

  auto weights = Eigen::VectorXd(model_.nv);
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    weights[i] =
        params_.nullspace.weights.joints_map.at(params_.joints.at(i)).value;
  }
  nullspace_stiffness.diagonal() << params_.nullspace.stiffness * weights;
  nullspace_damping.diagonal()
      << 2.0 * nullspace_stiffness.diagonal().cwiseSqrt();

  if (params_.nullspace.damping) {
    nullspace_damping.diagonal() = params_.nullspace.damping * weights;
  } else {
    nullspace_damping.diagonal() =
        2.0 * nullspace_stiffness.diagonal().cwiseSqrt();
  }
}

CallbackReturn CartesianController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  auto num_joints = params_.joints.size();
  for (size_t i = 0; i < num_joints; i++) {
    // Use cached joint IDs and models (moved from control loop to
    // configuration)
    const auto &joint = *joint_models_[i];

    q[i] = state_interfaces_[i].get_value();
    if (continous_joint_types.count(
            joint.shortname())) { // Then we are handling a continous
                                  // joint that is SO(2)
      q_pin[joint.idx_q()] = std::cos(q[i]);
      q_pin[joint.idx_q() + 1] = std::sin(q[i]);
    } else { // simple revolute joint case (handles ALL revolute types: RX, RY,
             // RZ, RevoluteUnaligned, etc.)
      q_pin[joint.idx_q()] = q[i];
    }

    q_ref[i] = state_interfaces_[i].get_value();

    dq[i] = state_interfaces_[num_joints + i].get_value();
    dq_ref[i] = state_interfaces_[num_joints + i].get_value();
  }

  pinocchio::forwardKinematics(model_, data_, q_pin, dq);
  pinocchio::updateFramePlacements(model_, data_);

  // Get end-effector pose in world frame (as Pinocchio provides it)
  ee_pose_world_ = data_.oMf[ee_frame_id_];

  // Compute base frame pose in world frame (should be constant, verified in on_configure)
  base_frame_pose_world_ = data_.oMf[base_frame_id_];

  RCLCPP_INFO(get_node()->get_logger(),
              "Base frame pose in world: pos=[%.3f, %.3f, %.3f]",
              base_frame_pose_world_.translation().x(),
              base_frame_pose_world_.translation().y(),
              base_frame_pose_world_.translation().z());

  // Initialize target to current pose in WORLD frame
  target_pose_ = ee_pose_world_;
  target_position_ = target_pose_.translation();
  target_orientation_ = Eigen::Quaterniond(target_pose_.rotation());

  // Log initial state to verify data quality
  RCLCPP_INFO(get_node()->get_logger(),
              "on_activate: Initial joint positions: [%.3f, %.3f, %.3f, %.3f, "
              "%.3f, %.3f, %.3f]",
              q[0], q[1], q[2], q[3], q[4], q[5], q[6]);
  RCLCPP_INFO(
      get_node()->get_logger(),
      "on_activate: Initial end-effector position (world): [%.3f, %.3f, %.3f]",
      target_position_.x(), target_position_.y(), target_position_.z());
  RCLCPP_INFO(get_node()->get_logger(),
              "on_activate: Initial end-effector orientation (world, quat): "
              "[%.3f, %.3f, %.3f, %.3f]",
              target_orientation_.w(), target_orientation_.x(),
              target_orientation_.y(), target_orientation_.z());

  // Read current effort state from hardware for bumpless transfer
  Eigen::VectorXd current_effort = Eigen::VectorXd::Zero(num_joints);
  bool has_nonzero_effort = false;
  for (size_t i = 0; i < num_joints; ++i) {
    current_effort[i] = state_interfaces_[2 * num_joints + i].get_value();
    if (std::abs(current_effort[i]) >
        0.01) { // Threshold for detecting active torques
      has_nonzero_effort = true;
    }
  }

  Eigen::VectorXd tau_init;

  // If switching from position mode (non-zero effort), seed with current effort
  // Otherwise compute holding torques from dynamics
  if (has_nonzero_effort) {
    RCLCPP_INFO(get_node()->get_logger(),
                "on_activate: Detected active torques from previous mode, "
                "using for bumpless transfer");
    tau_init = current_effort;
  } else {
    RCLCPP_INFO(get_node()->get_logger(),
                "on_activate: Computing holding torques from dynamics");
    // Compute initial control torques to hold current position when switching
    // to MIT mode This prevents the arm from falling by computing the full
    // control law at activation
    tau_init =
        computeControlTorques(ee_pose_world_, // current pose in world frame
                              target_pose_,  // target pose (same as current, in world frame)
                              q,             // joint positions
                              q_pin,         // pinocchio joint positions
                              dq, // joint velocities (should be near zero)
                              get_node()->get_clock()->now() // current time
        );
  }

  // Apply torque rate limiting and safety limits
  if (params_.limit_torques && tau_previous.size() > 0) {
    tau_init =
        saturateTorqueRate(tau_init, tau_previous, params_.max_delta_tau);
  }

  // Apply torque limits if configured
  if (tau_limits.size() > 0) {
    tau_init = tau_init.cwiseMin(tau_limits).cwiseMax(-tau_limits);
  }

  tau_previous = tau_init;
  tau_d = tau_init;

  RCLCPP_INFO(get_node()->get_logger(),
              "on_activate: Initialized with%s torques: [%.3f, %.3f, %.3f, "
              "%.3f, %.3f, %.3f, %.3f]",
              has_nonzero_effort ? " current effort" : " computed holding",
              tau_init[0], tau_init[1], tau_init[2], tau_init[3], tau_init[4],
              tau_init[5], tau_init[6]);

  // Open CSV log file if logging is enabled
  if (params_.log.enabled) {
    // Store the start time for consistent timestamp calculations
    csv_log_start_time_ = get_node()->now();

    if (params_.log.use_async_logging) {
      // Use async logger for better real-time performance
      csv_logger_ = std::make_unique<AsyncCSVLogger>(
          get_node()->get_name(), get_node()->get_logger());

      RCLCPP_INFO(get_node()->get_logger(),
                  "Using ASYNC CSV logging for improved real-time performance");
    } else {
      // Fallback to synchronous logger
      csv_logger_ = std::make_unique<ControllerCSVLogger>(
          get_node()->get_name(), get_node()->get_logger());

      RCLCPP_WARN(
          get_node()->get_logger(),
          "Using SYNCHRONOUS CSV logging - may impact real-time performance!");
    }

    if (!csv_logger_->initialize(num_joints, csv_log_start_time_)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Failed to initialize CSV logger");
      return CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(get_node()->get_logger(), "Controller activated.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // Close CSV logger if it was opened
  if (csv_logger_) {
    csv_logger_->close();
    csv_logger_.reset();
  }

  return CallbackReturn::SUCCESS;
}

void CartesianController::parse_target_pose_() {
  auto msg = *target_pose_buffer_.readFromRT();

  // Create SE3 from incoming pose (in base_frame coordinates)
  Eigen::Vector3d pose_in_base;
  pose_in_base << msg->pose.position.x, msg->pose.position.y,
      msg->pose.position.z;

  Eigen::Quaterniond quat_in_base(
      msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z);

  pinocchio::SE3 target_in_base(quat_in_base.toRotationMatrix(), pose_in_base);

  // Transform target from base frame to world frame for consistent computation
  // target_pose_ will be stored in world frame
  // We need to get the current base_frame_pose_world_ from forward kinematics
  base_frame_pose_world_ = data_.oMf[base_frame_id_];
  target_pose_ = base_frame_pose_world_ * target_in_base;

  // Store the individual components in world frame
  target_position_ = target_pose_.translation();
  target_orientation_ = Eigen::Quaterniond(target_pose_.rotation());
}

void CartesianController::parse_target_joint_() {
  auto msg = *target_joint_buffer_.readFromRT();
  if (msg->position.size()) {
    for (size_t i = 0; i < msg->position.size(); ++i) {
      q_ref[i] = msg->position[i];
    }
  }
  if (msg->velocity.size()) {
    for (size_t i = 0; i < msg->position.size(); ++i) {
      dq_ref[i] = msg->velocity[i];
    }
  }
}

void CartesianController::parse_target_wrench_() {
  auto msg = *target_wrench_buffer_.readFromRT();
  target_wrench_ << msg->wrench.force.x, msg->wrench.force.y,
      msg->wrench.force.z, msg->wrench.torque.x, msg->wrench.torque.y,
      msg->wrench.torque.z;
}

void CartesianController::log_debug_info(const rclcpp::Time &time,
                                         const rclcpp::Time &loop_start_time) {
  if (!params_.log.enabled) {
    return;
  }
  if (params_.log.robot_state) {
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), DEBUG_LOG_THROTTLE_MS,
                                "nq: " << model_.nq << ", nv: " << model_.nv);

    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), DEBUG_LOG_THROTTLE_MS,
                                "end_effector_pos (world): "
                                    << ee_pose_world_.translation());
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), DEBUG_LOG_THROTTLE_MS,
                                "q: " << q.transpose());
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), DEBUG_LOG_THROTTLE_MS,
                                "q_pin: " << q_pin.transpose());
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), DEBUG_LOG_THROTTLE_MS,
                                "dq: " << dq.transpose());
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), DEBUG_LOG_THROTTLE_MS,
                                "J: " << J);
  }

  if (params_.log.control_values) {
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), DEBUG_LOG_THROTTLE_MS,
                                "error: " << error.transpose());
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), DEBUG_LOG_THROTTLE_MS,
                                "max_delta: " << max_delta_.transpose());
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), DEBUG_LOG_THROTTLE_MS,
                                "q_ref: " << q_ref.transpose());
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), DEBUG_LOG_THROTTLE_MS,
                                "dq_ref: " << dq_ref.transpose());
  }

  if (params_.log.controller_parameters) {
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), DEBUG_LOG_THROTTLE_MS,
                                "stiffness: " << stiffness);
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), DEBUG_LOG_THROTTLE_MS,
                                "damping: " << damping);
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), DEBUG_LOG_THROTTLE_MS,
                                "nullspace_stiffness: " << nullspace_stiffness);
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), DEBUG_LOG_THROTTLE_MS,
                                "nullspace_damping: " << nullspace_damping);
  }

  if (params_.log.limits) {
    RCLCPP_INFO_STREAM_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(),
        DEBUG_LOG_THROTTLE_MS,
        "joint_limits: " << model_.lowerPositionLimit.transpose() << ", "
                         << model_.upperPositionLimit.transpose());
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), DEBUG_LOG_THROTTLE_MS,
                                "velocity_limits: " << model_.velocityLimit);
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), DEBUG_LOG_THROTTLE_MS,
                                "effort_limits: " << model_.effortLimit);
  }

  if (params_.log.computed_torques) {
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), DEBUG_LOG_THROTTLE_MS,
                                "tau_task: " << tau_task.transpose());
    RCLCPP_INFO_STREAM_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(),
        DEBUG_LOG_THROTTLE_MS,
        "tau_joint_limits: " << tau_joint_limits.transpose());
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), DEBUG_LOG_THROTTLE_MS,
                                "tau_nullspace: " << tau_nullspace.transpose());
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), DEBUG_LOG_THROTTLE_MS,
                                "tau_friction: " << tau_friction.transpose());
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), DEBUG_LOG_THROTTLE_MS,
                                "tau_coriolis: " << tau_coriolis.transpose());
  }

  if (params_.log.dynamic_params) {
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), DEBUG_LOG_THROTTLE_MS,
                                "M: " << data_.M);
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), DEBUG_LOG_THROTTLE_MS,
                                "Minv: " << data_.Minv);
    RCLCPP_INFO_STREAM_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(),
        DEBUG_LOG_THROTTLE_MS, "nullspace projector: " << nullspace_projection);
  }

  if (params_.log.timing) {

    auto t_end = get_node()->get_clock()->now();
    RCLCPP_INFO_STREAM_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(),
        TIMING_LOG_THROTTLE_MS,
        "Control loop needed: "
            << (t_end.nanoseconds() - time.nanoseconds()) * 1e-6 << " ms");
  }

  // Write CSV data if logging is enabled
  if (csv_logger_ && csv_logger_->isLoggingEnabled()) {
    ControllerLogData log_data;

    // Populate log data structure
    // Fix: Calculate timestamp correctly as time since start
    log_data.timestamp = (time - csv_log_start_time_).seconds();
    log_data.task_force_P = task_force_P_;
    log_data.task_force_D = task_force_D_;
    log_data.task_force_total = task_force_total_;
    log_data.task_velocity = task_velocity_;

    log_data.tau_task = tau_task;
    log_data.tau_nullspace = tau_nullspace;
    log_data.tau_joint_limits = tau_joint_limits;
    log_data.tau_friction = tau_friction;
    log_data.tau_coriolis = tau_coriolis;
    log_data.tau_gravity = tau_gravity;
    log_data.tau_wrench = tau_wrench;
    log_data.tau_total = tau_d;

    log_data.error = error;
    log_data.error_rot_magnitude = error.tail(3).norm();
    log_data.error_pos_magnitude = error.head(3).norm();

    // Log poses in world/universe frame (where computation happens)
    log_data.current_pose = ee_pose_world_;
    log_data.target_pose = target_pose_;  // Already in world frame

    for (int i = 0; i < 6; ++i) {
      log_data.stiffness_diag[i] = stiffness(i, i);
      log_data.damping_diag[i] = damping(i, i);
    }

    log_data.q_raw = q_raw;
    log_data.q_filtered = q;
    log_data.dq_raw = dq_raw;
    log_data.dq_filtered = dq;

    log_data.filter_q = params_.filter.q;
    log_data.filter_dq = params_.filter.dq;
    log_data.filter_output_torque = params_.filter.output_torque;

    // Calculate control loop duration
    auto loop_end_time = get_node()->get_clock()->now();
    log_data.loop_duration_ms =
        (loop_end_time - loop_start_time).nanoseconds() * 1e-6;

    // Log the data using the unified interface
    csv_logger_->logData(log_data, time);
  }
}

bool CartesianController::check_topic_publisher_count(
    const std::string &topic_name) {
  auto topic_info = get_node()->get_publishers_info_by_topic(topic_name);
  size_t publisher_count = topic_info.size();

  if (publisher_count > max_allowed_publishers_) {
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "Topic '%s' has %zu publishers (expected max: %zu). Multiple command "
        "sources detected!",
        topic_name.c_str(), publisher_count, max_allowed_publishers_);

    if (!multiple_publishers_detected_) {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "SAFETY WARNING: Multiple publishers detected on topic '%s'! "
          "Ignoring commands from this topic to prevent conflicting control "
          "signals.",
          topic_name.c_str());
      multiple_publishers_detected_ = true;
    }
    return false;
  }

  if (multiple_publishers_detected_ &&
      publisher_count <= max_allowed_publishers_) {
    RCLCPP_INFO(get_node()->get_logger(),
                "Publisher conflict resolved on topic '%s'. Resuming message "
                "processing.",
                topic_name.c_str());
    multiple_publishers_detected_ = false;
  }

  return true;
}

Eigen::VectorXd CartesianController::computeControlTorques(
    const pinocchio::SE3 &current_pose, const pinocchio::SE3 &target_pose,
    const Eigen::VectorXd &q, const Eigen::VectorXd &q_pin,
    const Eigen::VectorXd &dq, [[maybe_unused]] const rclcpp::Time &time) {

  // Compute pose error in WORLD frame (both poses are in world frame)
  Eigen::VectorXd error = Eigen::VectorXd::Zero(6);
  error.head(3) = target_pose.translation() - current_pose.translation();
  error.tail(3) = pinocchio::log3(target_pose.rotation() *
                                  current_pose.rotation().transpose());

  // Apply error limits if configured
  if (params_.limit_error) {
    Eigen::VectorXd max_delta(6);
    max_delta << params_.task.error_clip.x, params_.task.error_clip.y,
        params_.task.error_clip.z, params_.task.error_clip.rx,
        params_.task.error_clip.ry, params_.task.error_clip.rz;
    error = error.cwiseMax(-max_delta).cwiseMin(max_delta);
  }

  // Compute Jacobian in WORLD frame
  Eigen::MatrixXd J_local = Eigen::MatrixXd::Zero(6, model_.nv);
  pinocchio::computeFrameJacobian(model_, data_, q_pin, ee_frame_id_,
                                  pinocchio::ReferenceFrame::WORLD, J_local);

  // Compute nullspace projection
  Eigen::MatrixXd J_pinv =
      pseudo_inverse(J_local, params_.nullspace.regularization);
  Eigen::MatrixXd nullspace_proj;

  if (params_.nullspace.projector_type == "dynamic") {
    pinocchio::computeMinverse(model_, data_, q_pin);
    auto Mx_inv = J_local * data_.Minv * J_local.transpose();
    auto Mx = pseudo_inverse(Mx_inv);
    auto J_bar = data_.Minv * J_local.transpose() * Mx;
    nullspace_proj = Id_nv_ - J_local.transpose() * J_bar.transpose();
  } else if (params_.nullspace.projector_type == "kinematic") {
    nullspace_proj = Id_nv_ - J_pinv * J_local;
  } else {
    nullspace_proj = Id_nv_;
  }

  // Compute task-space control torques
  Eigen::Vector<double, 6> task_force_P_local = stiffness * error;
  Eigen::Vector<double, 6> task_velocity = J_local * dq;
  Eigen::Vector<double, 6> task_force_D_local = damping * task_velocity;
  Eigen::Vector<double, 6> task_force_total_local;

  Eigen::VectorXd tau_task_local;
  if (params_.use_operational_space) {
    pinocchio::computeMinverse(model_, data_, q_pin);
    auto Mx_inv = J_local * data_.Minv * J_local.transpose();
    auto Mx = pseudo_inverse(Mx_inv);
    task_force_total_local = Mx * (task_force_P_local - task_force_D_local);
    tau_task_local = J_local.transpose() * task_force_total_local;
  } else {
    task_force_total_local = task_force_P_local - task_force_D_local;
    tau_task_local = J_local.transpose() * task_force_total_local;
  }

  // Compute nullspace torques
  Eigen::VectorXd tau_secondary =
      nullspace_stiffness * (q_ref - q) + nullspace_damping * (dq_ref - dq);
  Eigen::VectorXd tau_nullspace_local = nullspace_proj * tau_secondary;
  tau_nullspace_local = tau_nullspace_local.cwiseMin(params_.nullspace.max_tau)
                            .cwiseMax(-params_.nullspace.max_tau);

  // Compute gravity compensation
  Eigen::VectorXd tau_gravity_local =
      params_.use_gravity_compensation
          ? Eigen::VectorXd(
                params_.gravity_scale *
                pinocchio::computeGeneralizedGravity(model_, data_, q_pin))
          : Eigen::VectorXd::Zero(model_.nv);

  // Compute Coriolis compensation
  Eigen::VectorXd tau_coriolis_local = Eigen::VectorXd::Zero(model_.nv);
  if (params_.use_coriolis_compensation) {
    pinocchio::computeAllTerms(model_, data_, q_pin, dq);
    tau_coriolis_local =
        pinocchio::computeCoriolisMatrix(model_, data_, q_pin, dq) * dq;
  }

  // Compute friction compensation
  Eigen::VectorXd tau_friction_local = params_.use_friction
                                           ? get_friction(dq, fp1, fp2, fp3)
                                           : Eigen::VectorXd::Zero(model_.nv);

  // Compute joint limit avoidance
  Eigen::VectorXd tau_joint_limits_local = Eigen::VectorXd::Zero(model_.nv);
  if (model_.nq == model_.nv && params_.joint_limit_avoidance.enable) {
    tau_joint_limits_local = get_joint_limit_torque(
        q, model_.lowerPositionLimit, model_.upperPositionLimit,
        params_.joint_limit_avoidance.safe_range,
        params_.joint_limit_avoidance.max_torque);
  }

  // Compute wrench contribution
  Eigen::VectorXd tau_wrench_local = J_local.transpose() * target_wrench_;

  // Combine all torque components
  Eigen::VectorXd tau_total = tau_task_local + tau_nullspace_local +
                              tau_friction_local + tau_coriolis_local +
                              tau_gravity_local + tau_joint_limits_local +
                              tau_wrench_local;

  return tau_total;
}

} // namespace crisp_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(crisp_controllers::CartesianController,
                       controller_interface::ControllerInterface)
