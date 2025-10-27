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
  return config;
}

controller_interface::return_type
CartesianController::update(const rclcpp::Time &time,
                            const rclcpp::Duration & /*period*/) {

  size_t num_joints = params_.joints.size();

  // Log first update to compare with on_activate values
  static bool first_update = true;
  if (first_update) {
    RCLCPP_INFO(get_node()->get_logger(), "First update() call - checking joint positions...");
  }

  for (size_t i = 0; i < num_joints; i++) {

    // TODO: later it might be better to get this thing prepared in the
    // configuration part (not in the control loop)
    auto joint_name = params_.joints[i];
    auto joint_id =
        model_.getJointId(joint_name); // pinocchio joind id might be different
    auto joint = model_.joints[joint_id];

    /*q[i] = exponential_moving_average(q[i],
     * state_interfaces_[i].get_value(),*/
    /*                                  params_.filter.q);*/
    q[i] = state_interfaces_[i].get_value();
    if (continous_joint_types.count(
            joint.shortname())) { // Then we are handling a continous
                                  // joint that is SO(2)
      q_pin[joint.idx_q()] = std::cos(q[i]);
      q_pin[joint.idx_q() + 1] = std::sin(q[i]);
    } else { // simple revolute joint case
      q_pin[joint.idx_q()] = q[i];
    }
    /*dq[i] = exponential_moving_average(*/
    /*    dq[i], state_interfaces_[num_joints + i].get_value(),*/
    /*    params_.filter.dq);*/
    dq[i] = state_interfaces_[num_joints + i].get_value();
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

  pinocchio::SE3 new_target_pose =
      pinocchio::SE3(target_orientation_.toRotationMatrix(), target_position_);

  target_pose_ = pinocchio::exp6(exponential_moving_average(
      pinocchio::log6(target_pose_), pinocchio::log6(new_target_pose),
      params_.filter.target_pose));

  /*target_pose_ = pinocchio::SE3(target_orientation_.toRotationMatrix(),
   * target_position_);*/
  end_effector_pose = data_.oMf[end_effector_frame_id];

  // Log first update to verify state interface data
  if (first_update) {
    RCLCPP_INFO(get_node()->get_logger(),
                "First update: Joint positions: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                q[0], q[1], q[2], q[3], q[4], q[5], q[6]);
    Eigen::Vector3d current_pos = end_effector_pose.translation();
    Eigen::Quaterniond current_quat(end_effector_pose.rotation());
    RCLCPP_INFO(get_node()->get_logger(),
                "First update: Current end-effector position: [%.3f, %.3f, %.3f]",
                current_pos.x(), current_pos.y(), current_pos.z());
    RCLCPP_INFO(get_node()->get_logger(),
                "First update: Target end-effector position: [%.3f, %.3f, %.3f]",
                target_position_.x(), target_position_.y(), target_position_.z());
    first_update = false;
  }

  // We consider translation and rotation separately to avoid unatural screw
  // motions
  if (params_.use_local_jacobian) {
    error.head(3) =
        end_effector_pose.rotation().transpose() *
        (target_pose_.translation() - end_effector_pose.translation());
    error.tail(3) = pinocchio::log3(end_effector_pose.rotation().transpose() *
                                    target_pose_.rotation());
  } else {
    error.head(3) =
        target_pose_.translation() - end_effector_pose.translation();
    error.tail(3) = pinocchio::log3(target_pose_.rotation() *
                                    end_effector_pose.rotation().transpose());
  }

  if (params_.limit_error) {
    max_delta_ << params_.task.error_clip.x, params_.task.error_clip.y,
        params_.task.error_clip.z, params_.task.error_clip.rx,
        params_.task.error_clip.ry, params_.task.error_clip.rz;
    error = error.cwiseMax(-max_delta_).cwiseMin(max_delta_);
  }

  // Log error every 100 cycles (~1Hz at 100Hz update rate)
  static int log_counter = 0;
  if (log_counter++ % 100 == 0) {
    RCLCPP_INFO(get_node()->get_logger(),
                "Cartesian error: pos=[%.4f, %.4f, %.4f]m, ori=[%.4f, %.4f, %.4f]rad",
                error[0], error[1], error[2], error[3], error[4], error[5]);
  }

  J.setZero();
  auto reference_frame = params_.use_local_jacobian
                             ? pinocchio::ReferenceFrame::LOCAL
                             : pinocchio::ReferenceFrame::WORLD;
  pinocchio::computeFrameJacobian(model_, data_, q_pin, end_effector_frame_id,
                                  reference_frame, J);

  Eigen::MatrixXd J_pinv(model_.nv, 6);
  J_pinv = pseudo_inverse(J, params_.nullspace.regularization);
  Eigen::MatrixXd Id_nv = Eigen::MatrixXd::Identity(model_.nv, model_.nv);

  if (params_.nullspace.projector_type == "dynamic") {
    pinocchio::computeMinverse(model_, data_, q_pin);
    auto Mx_inv = J * data_.Minv * J.transpose();
    auto Mx = pseudo_inverse(Mx_inv);
    auto J_bar = data_.Minv * J.transpose() * Mx;
    nullspace_projection = Id_nv - J.transpose() * J_bar.transpose();
  } else if (params_.nullspace.projector_type == "kinematic") {
    nullspace_projection = Id_nv - J_pinv * J;
  } else if (params_.nullspace.projector_type == "none") {
    nullspace_projection = Eigen::MatrixXd::Identity(model_.nv, model_.nv);
  } else {
    RCLCPP_ERROR_STREAM_ONCE(get_node()->get_logger(),
                             "Unknown nullspace projector type: "
                                 << params_.nullspace.projector_type);
    return controller_interface::return_type::ERROR;
  }

  if (params_.use_operational_space) {

    pinocchio::computeMinverse(model_, data_, q_pin);
    auto Mx_inv = J * data_.Minv * J.transpose();
    auto Mx = pseudo_inverse(Mx_inv);

    tau_task << J.transpose() * Mx * (stiffness * error - damping * (J * dq));
  } else {
    tau_task << J.transpose() * (stiffness * error - damping * (J * dq));
  }

  if (model_.nq != model_.nv) {
    // TODO: Then we have some continouts joints, not being handled for now
    tau_joint_limits = Eigen::VectorXd::Zero(model_.nv);
  } else if (params_.joint_limit_avoidance.enable) {
    tau_joint_limits = get_joint_limit_torque(
        q, model_.lowerPositionLimit, model_.upperPositionLimit,
        params_.joint_limit_avoidance.safe_range,
        params_.joint_limit_avoidance.max_torque);
  } else {
    tau_joint_limits = Eigen::VectorXd::Zero(model_.nv);
  }

  tau_secondary << nullspace_stiffness * (q_ref - q) +
                       nullspace_damping * (dq_ref - dq);

  tau_nullspace << nullspace_projection * tau_secondary;
  tau_nullspace = tau_nullspace.cwiseMin(params_.nullspace.max_tau)
                      .cwiseMax(-params_.nullspace.max_tau);

  tau_friction = params_.use_friction ? get_friction(dq, fp1, fp2, fp3)
                                      : Eigen::VectorXd::Zero(model_.nv);

  if (params_.use_coriolis_compensation) {
    pinocchio::computeAllTerms(model_, data_, q_pin, dq);
    tau_coriolis =
        pinocchio::computeCoriolisMatrix(model_, data_, q_pin, dq) * dq;
  } else {
    tau_coriolis = Eigen::VectorXd::Zero(model_.nv);
  }

  tau_gravity = params_.use_gravity_compensation
                    ? pinocchio::computeGeneralizedGravity(model_, data_, q_pin)
                    : Eigen::VectorXd::Zero(model_.nv);

  tau_wrench << J.transpose() * target_wrench_;

  tau_d << tau_task + tau_nullspace + tau_friction + tau_coriolis +
               tau_gravity + tau_joint_limits + tau_wrench;

  if (params_.limit_torques) {
    tau_d = saturateTorqueRate(tau_d, tau_previous, params_.max_delta_tau);
  }
  /*tau_d = exponential_moving_average(tau_d, tau_previous,*/
  /*                                   params_.filter.output_torque);*/

  // Log commanded torques every 100 cycles (~1Hz at 100Hz update rate)
  static int torque_log_counter = 0;
  if (torque_log_counter++ % 100 == 0) {
    RCLCPP_INFO(get_node()->get_logger(),
                "Commanded torques: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f] Nm (stop_commands=%s)",
                tau_d[0], tau_d[1], tau_d[2], tau_d[3], tau_d[4], tau_d[5], tau_d[6],
                params_.stop_commands ? "TRUE" : "FALSE");
  }

  if (not params_.stop_commands) {
    for (size_t i = 0; i < num_joints; ++i) {
      command_interfaces_[i].set_value(tau_d[i]);
    }
  } else {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                         "stop_commands is TRUE - not sending torques to hardware!");
  }

  tau_previous = tau_d;

  params_listener_->refresh_dynamic_parameters();
  params_ = params_listener_->get_params();
  setStiffnessAndDamping();

  log_debug_info(time);

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

  RCLCPP_INFO(get_node()->get_logger(), "Starting CartesianController configuration...");

  // Create a separate node and executor to avoid deadlock with controller_manager's executor
  auto param_node = std::make_shared<rclcpp::Node>(
      "cartesian_controller_param_client",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(false));

  auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
      param_node, "robot_state_publisher");

  RCLCPP_INFO(get_node()->get_logger(), "Waiting for robot_state_publisher service...");
  parameters_client->wait_for_service();
  RCLCPP_INFO(get_node()->get_logger(), "robot_state_publisher service found!");

  RCLCPP_INFO(get_node()->get_logger(), "Getting robot_description parameter...");
  auto future = parameters_client->get_parameters({"robot_description"});

  // Spin the separate node to process the async response
  while (rclcpp::ok() && future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
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

  RCLCPP_INFO(get_node()->get_logger(), "Building Pinocchio model from URDF...");
  pinocchio::Model raw_model_;
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

  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "Looking for end effector frame: " << params_.end_effector_frame);

  if (!model_.existFrame(params_.end_effector_frame)) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        "End effector frame '" << params_.end_effector_frame << "' not found in model!");
    RCLCPP_ERROR(get_node()->get_logger(), "Available frames:");
    for (size_t i = 0; i < model_.frames.size(); i++) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  - " << model_.frames[i].name);
    }
    return CallbackReturn::ERROR;
  }

  end_effector_frame_id = model_.getFrameId(params_.end_effector_frame);
  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "Found end effector frame with ID: " << end_effector_frame_id);

  q = Eigen::VectorXd::Zero(model_.nv);
  q_pin = Eigen::VectorXd::Zero(model_.nq);
  dq = Eigen::VectorXd::Zero(model_.nv);
  q_ref = Eigen::VectorXd::Zero(model_.nv);
  dq_ref = Eigen::VectorXd::Zero(model_.nv);
  tau_previous = Eigen::VectorXd::Zero(model_.nv);
  J = Eigen::MatrixXd::Zero(6, model_.nv);

  // Map the friction parameters to Eigen vectors
  fp1 = Eigen::Map<Eigen::VectorXd>(params_.friction.fp1.data(), model_.nv);
  fp2 = Eigen::Map<Eigen::VectorXd>(params_.friction.fp2.data(), model_.nv);
  fp3 = Eigen::Map<Eigen::VectorXd>(params_.friction.fp3.data(), model_.nv);

  nullspace_stiffness = Eigen::MatrixXd::Zero(model_.nv, model_.nv);
  nullspace_damping = Eigen::MatrixXd::Zero(model_.nv, model_.nv);

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
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Ignoring target_pose message due to multiple publishers detected!");
      return;
    }
    RCLCPP_INFO(get_node()->get_logger(),
                "Received target pose: frame=%s, pos=[%.3f, %.3f, %.3f], ori=[%.3f, %.3f, %.3f, %.3f]",
                msg->header.frame_id.c_str(),
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                msg->pose.orientation.w, msg->pose.orientation.x,
                msg->pose.orientation.y, msg->pose.orientation.z);
    target_pose_buffer_.writeFromNonRT(msg);
    new_target_pose_ = true;
  };

  auto target_joint_callback =
      [this](const std::shared_ptr<sensor_msgs::msg::JointState> msg) -> void {
    if (!check_topic_publisher_count("target_joint")) {
      RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
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

  RCLCPP_INFO(get_node()->get_logger(),
              "State interfaces and control vectors initialized.");

  RCLCPP_INFO(get_node()->get_logger(),
              "CartesianController configuration completed successfully!");

  return CallbackReturn::SUCCESS;
}

void CartesianController::setStiffnessAndDamping() {

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

    // TODO: later it might be better to get this thing prepared in the
    // configuration part (not in the control loop)
    auto joint_name = params_.joints[i];
    auto joint_id =
        model_.getJointId(joint_name); // pinocchio joind id might be different
    auto joint = model_.joints[joint_id];

    q[i] = state_interfaces_[i].get_value();
    if (continous_joint_types.count(
                   joint.shortname())) { // Then we are handling a continous
                                         // joint that is SO(2)
      q_pin[joint.idx_q()] = std::cos(q[i]);
      q_pin[joint.idx_q() + 1] = std::sin(q[i]);
    } else { // simple revolute joint case (handles ALL revolute types: RX, RY, RZ, RevoluteUnaligned, etc.)
      q_pin[joint.idx_q()] = q[i];
    }

    q_ref[i] = state_interfaces_[i].get_value();

    dq[i] = state_interfaces_[num_joints + i].get_value();
    dq_ref[i] = state_interfaces_[num_joints + i].get_value();
  }

  pinocchio::forwardKinematics(model_, data_, q_pin, dq);
  pinocchio::updateFramePlacements(model_, data_);

  end_effector_pose = data_.oMf[end_effector_frame_id];

  target_position_ = end_effector_pose.translation();
  target_orientation_ = Eigen::Quaterniond(end_effector_pose.rotation());
  target_pose_ =
      pinocchio::SE3(target_orientation_.toRotationMatrix(), target_position_);

  // Log initial state to verify data quality
  RCLCPP_INFO(get_node()->get_logger(),
              "on_activate: Initial joint positions: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
              q[0], q[1], q[2], q[3], q[4], q[5], q[6]);
  RCLCPP_INFO(get_node()->get_logger(),
              "on_activate: Computed initial end-effector position: [%.3f, %.3f, %.3f]",
              target_position_.x(), target_position_.y(), target_position_.z());
  RCLCPP_INFO(get_node()->get_logger(),
              "on_activate: Computed initial end-effector orientation (quat): [%.3f, %.3f, %.3f, %.3f]",
              target_orientation_.w(), target_orientation_.x(),
              target_orientation_.y(), target_orientation_.z());

  // Initialize commanded torques to zero to prevent jumps at startup
  tau_previous = Eigen::VectorXd::Zero(model_.nv);
  tau_d = Eigen::VectorXd::Zero(model_.nv);

  // Open CSV log file if logging is enabled
  if (params_.log.enabled) {
    // Get user workspace directory
    const char* user_ws = std::getenv("USER_WS");
    if (!user_ws) {
      RCLCPP_ERROR(get_node()->get_logger(), "USER_WS environment variable not set. Cannot create log directory.");
      return CallbackReturn::ERROR;
    }

    std::string log_dir = std::string(user_ws) + "/crisp_controller_logs";

    // Get timestamp with millisecond precision
    auto now = get_node()->now();
    int64_t timestamp_ms = now.seconds() * 1000 + now.nanoseconds() / 1000000;

    std::string log_filename = log_dir + "/" + get_node()->get_name() + "_" +
                              std::to_string(timestamp_ms) + ".csv";

    // Create directory if it doesn't exist
    std::filesystem::create_directories(log_dir);

    csv_log_file_.open(log_filename, std::ios::out);
    if (csv_log_file_.is_open()) {
      csv_logging_enabled_ = true;
      csv_log_start_time_ = get_node()->now();

      // Write CSV header
      csv_log_file_ << "timestamp";

      // Add torque columns for each joint
      for (auto i = 0u; i < num_joints; ++i) {
        csv_log_file_ << ",tau_task_" << i
                      << ",tau_nullspace_" << i
                      << ",tau_joint_limits_" << i
                      << ",tau_friction_" << i
                      << ",tau_coriolis_" << i
                      << ",tau_gravity_" << i
                      << ",tau_wrench_" << i
                      << ",tau_total_" << i;
      }

      // Add error columns (6 DOF: x, y, z, rx, ry, rz)
      csv_log_file_ << ",error_x,error_y,error_z,error_rx,error_ry,error_rz,error_xyz_norm";

      // Add pose columns (current and target)
      csv_log_file_ << ",current_x,current_y,current_z,current_qw,current_qx,current_qy,current_qz";
      csv_log_file_ << ",target_x,target_y,target_z,target_qw,target_qx,target_qy,target_qz";

      csv_log_file_ << std::endl;

      RCLCPP_INFO(get_node()->get_logger(), "CSV logging enabled: %s", log_filename.c_str());
    } else {
      csv_logging_enabled_ = false;
      RCLCPP_WARN(get_node()->get_logger(), "Failed to open CSV log file: %s", log_filename.c_str());
    }
  }

  RCLCPP_INFO(get_node()->get_logger(), "Controller activated.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // Close CSV log file if it was opened
  if (csv_logging_enabled_ && csv_log_file_.is_open()) {
    csv_log_file_.close();
    csv_logging_enabled_ = false;
    RCLCPP_INFO(get_node()->get_logger(), "CSV log file closed.");
  }
  return CallbackReturn::SUCCESS;
}

void CartesianController::parse_target_pose_() {
  auto msg = *target_pose_buffer_.readFromRT();
  target_position_ << msg->pose.position.x, msg->pose.position.y,
      msg->pose.position.z;
  target_orientation_ =
      Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                         msg->pose.orientation.y, msg->pose.orientation.z);
}

void CartesianController::parse_target_joint_() {
  auto msg = *target_joint_buffer_.readFromRT();
  if (msg->position.size()) {
    for (size_t i = 0; i < msg->position.size(); ++i) {
      q_ref[i] = msg->position[i];
    }
    /*filterJointValues(msg->name, msg->position, params_.joints, q_ref);*/
  }
  if (msg->velocity.size()) {
    for (size_t i = 0; i < msg->position.size(); ++i) {
      dq_ref[i] = msg->velocity[i];
    }
    /*filterJointValues(msg->name, msg->velocity, params_.joints, dq_ref);*/
  }
}

void CartesianController::parse_target_wrench_() {
  auto msg = *target_wrench_buffer_.readFromRT();
  target_wrench_ << msg->wrench.force.x, msg->wrench.force.y,
      msg->wrench.force.z, msg->wrench.torque.x, msg->wrench.torque.y,
      msg->wrench.torque.z;
}

void CartesianController::log_debug_info(const rclcpp::Time &time) {
  if (!params_.log.enabled) {
    return;
  }
  if (params_.log.robot_state) {
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), 1000,
                                "nq: " << model_.nq << ", nv: " << model_.nv);

    RCLCPP_INFO_STREAM_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "end_effector_pos" << end_effector_pose.translation());
    /*RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
     * *get_node()->get_clock(),*/
    /*                            1000, "end_effector_rot" <<
     * end_effector_pose.rotation());*/
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), 1000,
                                "q: " << q.transpose());
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), 1000,
                                "q_pin: " << q_pin.transpose());
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), 1000,
                                "dq: " << dq.transpose());
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), 1000, "J: " << J);
  }

  if (params_.log.control_values) {
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), 1000,
                                "error: " << error.transpose());
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), 1000,
                                "max_delta: " << max_delta_.transpose());
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), 1000,
                                "q_ref: " << q_ref.transpose());
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), 1000,
                                "dq_ref: " << dq_ref.transpose());
  }

  if (params_.log.controller_parameters) {
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), 1000,
                                "stiffness: " << stiffness);
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), 1000,
                                "damping: " << damping);
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), 1000,
                                "nullspace_stiffness: " << nullspace_stiffness);
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), 1000,
                                "nullspace_damping: " << nullspace_damping);
  }

  if (params_.log.limits) {
    RCLCPP_INFO_STREAM_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "joint_limits: " << model_.lowerPositionLimit.transpose() << ", "
                         << model_.upperPositionLimit.transpose());
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), 1000,
                                "velocity_limits: " << model_.velocityLimit);
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), 1000,
                                "effort_limits: " << model_.effortLimit);
  }

  if (params_.log.computed_torques) {
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), 1000,
                                "tau_task: " << tau_task.transpose());
    RCLCPP_INFO_STREAM_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "tau_joint_limits: " << tau_joint_limits.transpose());
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), 1000,
                                "tau_nullspace: " << tau_nullspace.transpose());
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), 1000,
                                "tau_friction: " << tau_friction.transpose());
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), 1000,
                                "tau_coriolis: " << tau_coriolis.transpose());
  }

  if (params_.log.dynamic_params) {
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), 1000,
                                "M: " << data_.M);
    /*RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
     * *get_node()->get_clock(),*/
    /*                            1000, "Mx: " << Mx);*/
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), 1000,
                                "Minv: " << data_.Minv);
    RCLCPP_INFO_STREAM_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "nullspace projector: " << nullspace_projection);
  }

  if (params_.log.timing) {

    auto t_end = get_node()->get_clock()->now();
    RCLCPP_INFO_STREAM_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "Control loop needed: "
            << (t_end.nanoseconds() - time.nanoseconds()) * 1e-6 << " ms");
  }

  // Write CSV data if logging is enabled
  if (csv_logging_enabled_ && csv_log_file_.is_open()) {
    // Write relative timestamp in seconds (from start of logging)
    double relative_time = (time - csv_log_start_time_).seconds();
    csv_log_file_ << relative_time;

    // Write torque components for each joint
    for (auto i = 0; i < tau_task.size(); ++i) {
      csv_log_file_ << "," << tau_task[i]
                    << "," << tau_nullspace[i]
                    << "," << tau_joint_limits[i]
                    << "," << tau_friction[i]
                    << "," << tau_coriolis[i]
                    << "," << tau_gravity[i]
                    << "," << tau_wrench[i]
                    << "," << tau_d[i];
    }

    // Write error (6 DOF: x, y, z, rx, ry, rz)
    for (auto i = 0; i < 6; ++i) {
      csv_log_file_ << "," << error[i];
    }

    // Write xyz error norm
    double error_xyz_norm = std::sqrt(error[0]*error[0] + error[1]*error[1] + error[2]*error[2]);
    csv_log_file_ << "," << error_xyz_norm;

    // Write current pose (position and orientation as quaternion)
    Eigen::Vector3d current_pos = end_effector_pose.translation();
    Eigen::Quaterniond current_quat(end_effector_pose.rotation());
    csv_log_file_ << "," << current_pos.x() << "," << current_pos.y() << "," << current_pos.z()
                  << "," << current_quat.w() << "," << current_quat.x()
                  << "," << current_quat.y() << "," << current_quat.z();

    // Write target pose (position and orientation as quaternion)
    Eigen::Vector3d target_pos = target_pose_.translation();
    Eigen::Quaterniond target_quat(target_pose_.rotation());
    csv_log_file_ << "," << target_pos.x() << "," << target_pos.y() << "," << target_pos.z()
                  << "," << target_quat.w() << "," << target_quat.x()
                  << "," << target_quat.y() << "," << target_quat.z();

    csv_log_file_ << std::endl;
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

} // namespace crisp_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(crisp_controllers::CartesianController,
                       controller_interface::ControllerInterface)
