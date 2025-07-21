#include <crisp_controllers/torque_feedback_controller.hpp>
#include <crisp_controllers/utils/friction_model.hpp>
#include <crisp_controllers/utils/pseudo_inverse.hpp>

#include <cmath>
#include <memory>
#include <set>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/urdf.hpp>


namespace crisp_controllers {

controller_interface::InterfaceConfiguration
TorqueFeedbackController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : joint_names_) {
    config.names.push_back(joint_name + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
TorqueFeedbackController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : joint_names_) {
    config.names.push_back(joint_name + "/position");
  }
  for (const auto &joint_name : joint_names_) {
    config.names.push_back(joint_name + "/velocity");
  }
  for (const auto &joint_name : joint_names_) {
    config.names.push_back(joint_name + "/effort");
  }
  return config;
}

controller_interface::return_type
TorqueFeedbackController::update(const rclcpp::Time & /*time*/,
                                 const rclcpp::Duration & /*period*/) {
  // Update joint states
  for (int i = 0; i < num_joints_; i++) {
    q_[i] = state_interfaces_[i].get_value();
    dq_[i] = state_interfaces_[num_joints_ + i].get_value();
    tau_[i] = state_interfaces_[2 * num_joints_ + i].get_value();
  }
  
  // Compute forward kinematics and jacobian
  pinocchio::forwardKinematics(model_, data_, q_, dq_);
  pinocchio::updateFramePlacements(model_, data_);
  
  J_.setZero();
  pinocchio::computeFrameJacobian(model_, data_, q_, end_effector_frame_id_,
                                  pinocchio::ReferenceFrame::LOCAL, J_);

  // Apply threshold to external torques based on vector magnitude
  Eigen::VectorXd tau_ext_thresholded = Eigen::VectorXd::Zero(num_joints_);
  double tau_ext_magnitude = tau_ext_.norm();
  if (tau_ext_magnitude > params_.torque_threshold) {
    tau_ext_thresholded = tau_ext_;
  }
  
  // Compute nullspace control to maintain initial joint positions
  Eigen::VectorXd q_error = q_ - q_init_;
  double nullspace_damping_computed = params_.nullspace.damping > 0 ? params_.nullspace.damping : 2.0 * sqrt(params_.nullspace.stiffness);
  
  Eigen::VectorXd tau_secondary = -params_.nullspace.stiffness * nullspace_weights_.cwiseProduct(q_error) - 
                                  nullspace_damping_computed * nullspace_weights_.cwiseProduct(dq_);
  
  // Compute nullspace projection based on projector type
  Eigen::MatrixXd Id_nv = Eigen::MatrixXd::Identity(model_.nv, model_.nv);
  
  if (params_.nullspace.projector_type == "dynamic") {
    pinocchio::computeMinverse(model_, data_, q_);
    auto Mx_inv = J_ * data_.Minv * J_.transpose();
    auto Mx = pseudo_inverse(Mx_inv);
    auto J_bar = data_.Minv * J_.transpose() * Mx;
    nullspace_projection_ = Id_nv - J_.transpose() * J_bar.transpose();
  } else if (params_.nullspace.projector_type == "kinematic") {
    Eigen::MatrixXd J_pinv = pseudo_inverse(J_, params_.nullspace.regularization);
    nullspace_projection_ = Id_nv - J_pinv * J_;
  } else if (params_.nullspace.projector_type == "none") {
    nullspace_projection_ = Id_nv;
  } else {
    RCLCPP_ERROR_STREAM_ONCE(get_node()->get_logger(),
                        "Unknown nullspace projector type: "
                            << params_.nullspace.projector_type);
    return controller_interface::return_type::ERROR;
  }
  
  // Apply nullspace projection
  Eigen::VectorXd tau_nullspace = nullspace_projection_ * tau_secondary;
  
  // Limit nullspace torques
  for (int i = 0; i < num_joints_; i++) {
    tau_nullspace[i] = std::max(-params_.nullspace.max_tau, std::min(params_.nullspace.max_tau, tau_nullspace[i]));
  }
  
  auto tau_d = -params_.k_fb * tau_ext_thresholded - params_.kd * dq_;
  auto tau_f = get_friction(dq_, friction_fp1_, friction_fp2_, friction_fp3_);


  for (int i = 0; i < num_joints_; i++) {
    command_interfaces_[i].set_value(tau_d[i] + tau_f[i] + tau_nullspace[i]);
  }

  params_listener_->refresh_dynamic_parameters();
  params_ = params_listener_->get_params();
  
  // Update nullspace weights
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    nullspace_weights_[i] = params_.nullspace.weights.joints_map.at(joint_names_[i]).value;
  }
  
  return controller_interface::return_type::OK;
}

CallbackReturn TorqueFeedbackController::on_init() {
  // Initialize parameters
  params_listener_ =
      std::make_shared<torque_feedback_controller::ParamListener>(get_node());
  params_listener_->refresh_dynamic_parameters();
  params_ = params_listener_->get_params();

  // Set basic parameters
  joint_names_ = params_.joints;
  num_joints_ = joint_names_.size();

  // Initialize state vectors
  q_ = Eigen::VectorXd::Zero(num_joints_);
  dq_ = Eigen::VectorXd::Zero(num_joints_);
  tau_ = Eigen::VectorXd::Zero(num_joints_);
  q_init_ = Eigen::VectorXd::Zero(num_joints_);

  tau_ext_ = Eigen::VectorXd::Zero(num_joints_);
  
  // Initialize nullspace weights
  nullspace_weights_ = Eigen::VectorXd::Ones(num_joints_);
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    nullspace_weights_[i] = params_.nullspace.weights.joints_map.at(joint_names_[i]).value;
  }
  
  // Initialize friction parameters as Eigen vectors
  friction_fp1_ = Eigen::Map<const Eigen::VectorXd>(params_.friction.fp1.data(), params_.friction.fp1.size());
  friction_fp2_ = Eigen::Map<const Eigen::VectorXd>(params_.friction.fp2.data(), params_.friction.fp2.size());
  friction_fp3_ = Eigen::Map<const Eigen::VectorXd>(params_.friction.fp3.data(), params_.friction.fp3.size());
  
  // Initialize nullspace projection matrix
  nullspace_projection_ = Eigen::MatrixXd::Identity(num_joints_, num_joints_);

  joint_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      params_.joint_source_topic,
      rclcpp::QoS(1).best_effort().keep_last(1).durability_volatile(),
      std::bind(&TorqueFeedbackController::target_joint_callback_, this,
                std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

CallbackReturn TorqueFeedbackController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  
  // Get robot description for pinocchio model
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

  // Build pinocchio model
  pinocchio::Model raw_model_;
  pinocchio::urdf::buildModelFromXML(robot_description_, raw_model_);

  // Check that the passed joints exist in the kinematic tree
  for (auto &joint : joint_names_) {
    if (not raw_model_.existJointName(joint)) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                          "Failed to configure because "
                              << joint
                              << " is not part of the kinematic tree but it "
                                 "has been passed in the parameters.");
      return CallbackReturn::ERROR;
    }
  }

  // Remove unused joints from the model
  std::vector<pinocchio::JointIndex> list_of_joints_to_lock_by_id;
  for (auto &joint : raw_model_.names) {
    if (std::find(joint_names_.begin(), joint_names_.end(), joint) ==
            joint_names_.end() and
        joint != "universe") {
      list_of_joints_to_lock_by_id.push_back(raw_model_.getJointId(joint));
    }
  }

  Eigen::VectorXd q_locked = Eigen::VectorXd::Zero(raw_model_.nq);
  model_ = pinocchio::buildReducedModel(raw_model_,
                                        list_of_joints_to_lock_by_id, q_locked);
  data_ = pinocchio::Data(model_);

  // Check for unsupported joint types (continuous joints not implemented)
  std::set<std::string> allowed_joint_types = {"JointModelRZ", "JointModelRevoluteUnaligned", "JointModelRX", "JointModelRY"};

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
              << "). Continuous joints are not implemented yet for torque feedback controller. "
              << "Only revolute joints are supported.");
      return CallbackReturn::ERROR;
    }
  }

  // Get end-effector frame ID
  end_effector_frame_id_ = model_.getFrameId(params_.end_effector_frame);
  
  // Initialize jacobian matrix
  J_ = Eigen::MatrixXd::Zero(6, model_.nv);

  return CallbackReturn::SUCCESS;
}

CallbackReturn TorqueFeedbackController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  for (int i = 0; i < num_joints_; i++) {
    q_[i] = state_interfaces_[i].get_value();
    dq_[i] = state_interfaces_[num_joints_ + i].get_value();
    tau_[i] = state_interfaces_[2 * num_joints_ + i].get_value();
    tau_ext_[i] = 0.0;
    
    // Record initial joint positions for nullspace control
    q_init_[i] = q_[i];
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TorqueFeedbackController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

void TorqueFeedbackController::target_joint_callback_(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  for (size_t i = 0; i < msg->effort.size(); i++) {
    if ((int)i < num_joints_) {
      tau_ext_[i] = msg->effort[i];
    }
  }
}

} // namespace crisp_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(crisp_controllers::TorqueFeedbackController,
                       controller_interface::ControllerInterface)
