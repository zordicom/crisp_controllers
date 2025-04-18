#include "crisp_controllers/utils/fiters.hpp"
#include "crisp_controllers/utils/torque_rate_saturation.hpp"
#include <crisp_controllers/pch.hpp>
#include <crisp_controllers/cartesian_impedance_controller.hpp>
#include <crisp_controllers/utils/friction_model.hpp>
#include <crisp_controllers/utils/joint_limits.hpp>
#include <crisp_controllers/utils/pseudo_inverse.hpp>

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hxx>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/fwd.hpp>
#include <rclcpp/logging.hpp>

namespace crisp_controllers {

controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : params_.joints) {
    config.names.push_back(joint_name + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
CartesianImpedanceController::state_interface_configuration() const {
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

controller_interface::return_type CartesianImpedanceController::update(
    const rclcpp::Time & time, const rclcpp::Duration & /*period*/) {

  size_t num_joints = params_.joints.size();
  for (size_t i = 0; i < num_joints; i++) {
    q[i] = exponential_moving_average(q[i], state_interfaces_[i].get_value(), params_.filter.q);
    dq[i] = exponential_moving_average(dq[i], state_interfaces_[num_joints + i].get_value(), params_.filter.dq);
    tau[i] = state_interfaces_[2 * num_joints + i].get_value();
  }

  pinocchio::forwardKinematics(model_, data_, q, dq);
  pinocchio::updateFramePlacements(model_, data_);

  pinocchio::SE3 new_target_pose = pinocchio::SE3(target_orientation_.toRotationMatrix(), target_position_);
  target_pose_ = pinocchio::exp6(exponential_moving_average(pinocchio::log6(target_pose_), pinocchio::log6(new_target_pose), params_.filter.target_pose));
  /*target_pose_ = pinocchio::SE3(target_orientation_.toRotationMatrix(), target_position_);*/

  end_effector_pose = data_.oMf[end_effector_frame_id];
  pinocchio::SE3 diff_pose = params_.use_local_jacobian ? end_effector_pose.inverse() * target_pose_ 
                                                        : target_pose_ * end_effector_pose.inverse();

  Eigen::VectorXd error(6);
  error = pinocchio::log6(diff_pose).toVector();

  auto max_delta_ =
      Eigen::Map<Eigen::VectorXd>(params_.max_delta.data(), 6);

  if (error.size() != max_delta_.size()) {
    RCLCPP_ERROR_ONCE(get_node()->get_logger(), "Size mismatch: error is %ld, max_delta_ is %ld", error.size(), max_delta_.size());
    return controller_interface::return_type::ERROR;
  }
  error.cwiseMin(max_delta_).cwiseMax(-max_delta_);

  J.setZero();
  auto reference_frame = params_.use_local_jacobian ? pinocchio::ReferenceFrame::LOCAL
                                                    : pinocchio::ReferenceFrame::WORLD;
  pinocchio::computeFrameJacobian(model_, data_, q, end_effector_frame_id,
                                  reference_frame, J);

  Eigen::MatrixXd J_pinv(model_.nv, 6);
  Eigen::MatrixXd Id_nv(model_.nv, model_.nv);

  pinocchio::computeMinverse(model_, data_, q);
  auto Mx_inv = J * data_.Minv * J.transpose();
  auto Mx = pseudoInverse(Mx_inv);

  J_pinv = pseudoInverse(J, params_.nullspace.regularization);

  Eigen::MatrixXd nullspace_projection;
  if (params_.nullspace.use_dynamic_projector) {
    nullspace_projection = Id_nv - J.transpose() * Mx * J * data_.Minv;
  } else {
    nullspace_projection = Id_nv - J_pinv * J;
  }

  // Now we compute all terms of the control law
  Eigen::VectorXd tau_task(model_.nv), tau_d(model_.nv),
      tau_secondary(model_.nv), tau_nullspace(model_.nv),
      tau_friction(model_.nv), tau_coriolis(model_.nv),
      tau_joint_limits(model_.nv), tau_random_noise(model_.nv);

  if (params_.use_operational_space) {

    /*RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),*/
    /*                           5000, "Using OSC");*/

    tau_task << J.transpose() * Mx * (stiffness * error - damping * (J * dq));
  } else {
    tau_task << J.transpose() * (stiffness * error - damping * (J * dq));
  }

  tau_joint_limits = get_joint_limit_torque(q, model_.lowerPositionLimit, model_.upperPositionLimit);

  tau_secondary << nullspace_stiffness * (q_ref - q) + nullspace_damping * (dq_ref - dq);

  tau_nullspace << nullspace_projection * tau_secondary;

  tau_friction = params_.use_friction ? get_friction(dq)
                                      : Eigen::VectorXd::Zero(model_.nv);

  tau_random_noise = params_.noise.add_random_noise ? (Eigen::VectorXd::Random(model_.nv) * params_.noise.amplitude).eval()
                                                    : Eigen::VectorXd::Zero(model_.nv);

  if (params_.use_coriolis_compensation) {
    pinocchio::computeAllTerms(model_, data_, q, dq);
    tau_coriolis = pinocchio::computeCoriolisMatrix(model_, data_, q, dq) * dq;
  } else {
    tau_coriolis = Eigen::VectorXd::Zero(model_.nv);
  }

  tau_d << tau_task + tau_nullspace + tau_friction + tau_coriolis +
               tau_joint_limits + tau_random_noise;

  if (params_.limit_torques) {
    tau_d = saturateTorqueRate(tau_d, tau_previous, params_.max_delta_tau);  // TODO: double check this
  } 


  for (size_t i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_d[i]);
  }

  tau_previous = tau_d;
  
  params_listener_->refresh_dynamic_parameters();
  params_ = params_listener_->get_params();
  setStiffnessAndDamping();

  if (params_.log.enabled) {
    if (params_.log.robot_state) {
      RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                  1000, "nq: " << model_.nq << ", nv: " << model_.nv);
      RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                  1000, "end_effector_pos" << end_effector_pose.translation());
      /*RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),*/
      /*                            1000, "end_effector_rot" << end_effector_pose.rotation());*/
      RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                  1000, "q: " << q.transpose());
      RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                  1000, "dq: " << dq.transpose());
      RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                  1000, "tau: " << tau.transpose());
      RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                  1000, "J: " << J);
    }

    if (params_.log.controller_parameters) {
      RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                  1000, "stiffness: " << stiffness);
      RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                  1000, "damping: " << damping);
      RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                  1000, "nullspace_stiffness: " << nullspace_stiffness);
      RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                  1000, "nullspace_damping: " << nullspace_damping);

    }

    if (params_.log.limits) {
      RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                  1000, "joint_limits: " << model_.lowerPositionLimit.transpose() << ", "
                                  << model_.upperPositionLimit.transpose());
      RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                  1000, "velocity_limits: " << model_.velocityLimit);
      RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                  1000, "effort_limits: " << model_.effortLimit);
    }

    if (params_.log.computed_torques) {
      RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                  1000, "tau_task: " << tau_task.transpose());
      RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                  1000, "tau_nullspace: " << tau_nullspace.transpose());
      RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                  1000, "tau_joint_limits: " << tau_joint_limits.transpose());
      RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                  1000, "tau_nullspace: " << tau_nullspace.transpose());
      RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                  1000, "tau_friction: " << tau_friction.transpose());
      RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                  1000, "tau_coriolis: " << tau_friction.transpose());

    }

    if (params_.log.dynamic_params) {
      RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                  1000, "M: " << data_.M);
      RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                  1000, "Mx: " << Mx);
      RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                  1000, "Minv: " << data_.Minv);
    }

    if (params_.log.timing) {

      auto t_end = get_node()->get_clock()->now();
      RCLCPP_INFO_STREAM_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "Control loop needed: " << (t_end.nanoseconds() - time.nanoseconds())*1e-6
                                  << " ms");
    }
  }



  return controller_interface::return_type::OK;
}

CallbackReturn CartesianImpedanceController::on_init() {
  params_listener_ =
      std::make_shared<cartesian_impedance_controller::ParamListener>(
          get_node());
  params_listener_->refresh_dynamic_parameters();
  params_ = params_listener_->get_params();

  pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "target_pose", rclcpp::QoS(1),
      std::bind(&CartesianImpedanceController::target_pose_callback_,
                this, std::placeholders::_1));

  joint_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      "target_joint", rclcpp::QoS(1),
      std::bind(&CartesianImpedanceController::target_joint_callback_,
                this, std::placeholders::_1));

  target_position_ = Eigen::Vector3d::Zero();
  target_orientation_ = Eigen::Quaterniond::Identity();

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
      get_node(), "robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();

  std::string robot_description_;
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to get robot_description parameter.");
    return CallbackReturn::ERROR;
  }

  pinocchio::urdf::buildModelFromXML(robot_description_, model_);
  data_ = pinocchio::Data(model_);

  end_effector_frame_id = model_.getFrameId(params_.end_effector_frame);
  q = Eigen::VectorXd::Zero(model_.nq);
  dq = Eigen::VectorXd::Zero(model_.nv);
  q_ref = Eigen::VectorXd::Zero(model_.nq);
  dq_ref = Eigen::VectorXd::Zero(model_.nv);
  tau = Eigen::VectorXd::Zero(model_.nv);
  tau_previous = Eigen::VectorXd::Zero(model_.nv);
  J = Eigen::MatrixXd::Zero(6, model_.nv);

  nullspace_stiffness = Eigen::MatrixXd::Zero(model_.nv, model_.nv);
  nullspace_damping = Eigen::MatrixXd::Zero(model_.nv, model_.nv);

  setStiffnessAndDamping();

  RCLCPP_INFO(get_node()->get_logger(), "State interfaces set up.");

  return CallbackReturn::SUCCESS;
}

void CartesianImpedanceController::setStiffnessAndDamping() {

  stiffness.setZero();
  stiffness.diagonal() << params_.task.k_pos_x, params_.task.k_pos_y, params_.task.k_pos_z,
      params_.task.k_rot_x, params_.task.k_rot_y, params_.task.k_rot_z;

  damping.setZero();
  damping.diagonal() << 2.0 * stiffness.diagonal().cwiseSqrt();

  nullspace_stiffness.setZero();
  nullspace_damping.setZero();

  if (params_.nullspace.weights.size() != params_.joints.size()) {
    RCLCPP_WARN_STREAM_ONCE(get_node()->get_logger(),
                 "Nullspace weights size does not match number of joints: weights.size(): "
                            << params_.nullspace.weights.size() << " joints.size(): " << params_.joints.size() 
                            << ", using default value of 1.0 for each joint.");
    nullspace_stiffness.diagonal() << params_.nullspace.stiffness * Eigen::VectorXd::Ones(params_.joints.size());
  } else {
    auto weights = Eigen::Map<Eigen::VectorXd>(params_.nullspace.weights.data(), model_.nv);
    nullspace_stiffness.diagonal() << params_.nullspace.stiffness * weights;
  }
  nullspace_damping.diagonal() << 2.0 * nullspace_stiffness.diagonal().cwiseSqrt();
}

CallbackReturn CartesianImpedanceController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  auto num_joints = params_.joints.size();
  for (size_t i = 0; i < num_joints; i++) {

    q[i] = state_interfaces_[i].get_value();
    q_ref[i] = state_interfaces_[i].get_value();

    dq[i] = state_interfaces_[num_joints + i].get_value();
    dq_ref[i] = state_interfaces_[num_joints + i].get_value();

  }

  pinocchio::forwardKinematics(model_, data_, q, dq);
  pinocchio::updateFramePlacements(model_, data_);

  end_effector_pose = data_.oMf[end_effector_frame_id];

  target_position_ = end_effector_pose.translation();
  target_orientation_ = Eigen::Quaterniond(end_effector_pose.rotation());
  target_pose_ =
      pinocchio::SE3(target_orientation_.toRotationMatrix(), target_position_);

  RCLCPP_INFO(get_node()->get_logger(), "Controller activated.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CartesianImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

void CartesianImpedanceController::target_pose_callback_(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {

  target_position_ << msg->pose.position.x, msg->pose.position.y,
      msg->pose.position.z;
  target_orientation_ =
      Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                         msg->pose.orientation.y, msg->pose.orientation.z);
}

void CartesianImpedanceController::target_joint_callback_(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  for (size_t i = 0; i < msg->position.size(); i++) {
    if (i < params_.joints.size()) {
      q_ref[i] = msg->position[i];
    }
  }
  for (size_t i = 0; i < msg->velocity.size(); i++) {
    if (i < params_.joints.size()) {
      dq_ref[i] = msg->velocity[i];
    }
  }
}

} // namespace crisp_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(
    crisp_controllers::CartesianImpedanceController,
    controller_interface::ControllerInterface)
