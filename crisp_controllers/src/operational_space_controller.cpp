#include "crisp_controllers/friction_model.hpp"
#include <crisp_controllers/operational_space_controller.hpp>
#include <crisp_controllers/pseudo_inverse.hpp>

#include <binders.h>
#include <cassert>
#include <cmath>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hxx>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/multibody/joint/joint-spherical.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/fwd.hpp>
#include <pinocchio/spatial/motion-base.hpp>
#include <rclcpp/logging.hpp>

using namespace std::chrono_literals;

typedef Eigen::Matrix<double, 6, 1> Vector6d;

namespace crisp_controllers {

controller_interface::InterfaceConfiguration
OperationalSpaceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : joint_names_) {
    config.names.push_back(joint_name + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
OperationalSpaceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // The first num_joints_ are the joint positions, the next num_joints_ are the
  // joint velocities, and the last num_joints_ are the joint torques
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
OperationalSpaceController::update(const rclcpp::Time & /*time*/,
                                   const rclcpp::Duration & /*period*/) {

  for (int i = 0; i < num_joints_; i++) {
    q[i] = state_interfaces_[i].get_value();
    dq[i] = state_interfaces_[num_joints_ + i].get_value();
    tau[i] = state_interfaces_[2 * num_joints_ + i].get_value();
  }

  pinocchio::forwardKinematics(model_, data_, q, dq);
  pinocchio::updateFramePlacements(model_, data_);
  target_pose_ =
      pinocchio::SE3(target_orientation_.toRotationMatrix(), target_position_);

  end_effector_pose = data_.oMf[end_effector_frame_id];
  pinocchio::SE3 diff_pose = end_effector_pose.inverse() * target_pose_;

  Eigen::VectorXd error(6);
  error.head(3) << diff_pose.translation();
  error.tail(3) << pinocchio::log3(diff_pose.rotation());

  error.cwiseMin(max_delta_).cwiseMax(-max_delta_);

  J.setZero();

  pinocchio::computeFrameJacobian(model_, data_, q, end_effector_frame_id,
                                  pinocchio::ReferenceFrame::LOCAL, J);

  Eigen::VectorXd tau_task(model_.nv), tau_d(model_.nv),
      tau_secondary(model_.nv), tau_nullspace(model_.nv),
      tau_friction(model_.nv);

  pinocchio::computeMinverse(model_, data_, q);

  Mx_inv = J * data_.Minv * J.transpose();

  Mx = pseudoInverse(Mx_inv);

  tau_task << J.transpose() * Mx * (stiffness * error - damping * (J * dq));

  tau_friction = params_.use_friction ? get_friction(dq)
                                      : Eigen::VectorXd::Zero(model_.nv);
  tau_d << tau_task + tau_friction;

  if (limit_torques_) {
    tau_d = tau_d.cwiseMin(max_torques_).cwiseMax(-max_torques_);
  }

  for (int i = 0; i < num_joints_; ++i) {
    command_interfaces_[i].set_value(tau_d[i]);
  }

  // Check if parameters changed and update with low pass filter
  params_ = params_listener_->get_params();
  const float alpha = 0.99;
  translational_stiffness = alpha * translational_stiffness +
                            (1 - alpha) * params_.translational_stiffness;
  rotational_stiffness =
      alpha * rotational_stiffness + (1 - alpha) * params_.rotational_stiffness;
  nullspace_stiffness_ =
      alpha * nullspace_stiffness_ + (1 - alpha) * params_.nullspace_stiffness;
  nullspace_damping_ = 2.0 * sqrt(nullspace_stiffness_);
  setStiffnessAndDamping();

  return controller_interface::return_type::OK;
}

CallbackReturn OperationalSpaceController::on_init() {
  params_listener_ =
      std::make_shared<operational_space_controller::ParamListener>(get_node());
  params_listener_->refresh_dynamic_parameters();
  params_ = params_listener_->get_params();

  joint_names_ = params_.joints;
  num_joints_ = joint_names_.size();
  end_effector_frame_ = params_.end_effector_frame;
  use_fake_hardware_ = params_.use_fake_hardware;
  translational_stiffness = params_.translational_stiffness;
  rotational_stiffness = params_.rotational_stiffness;
  nullspace_stiffness_ = params_.nullspace_stiffness;
  nullspace_damping_ = 2.0 * sqrt(nullspace_stiffness_);
  max_delta_ =
      Eigen::Map<Eigen::VectorXd>(params_.max_delta.data(), num_joints_);
  limit_torques_ = params_.limit_torques;

  pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "target_pose", rclcpp::QoS(1),
      std::bind(&OperationalSpaceController::target_pose_callback_, this,
                std::placeholders::_1));

  target_position_ = Eigen::Vector3d::Zero();
  target_orientation_ = Eigen::Quaterniond::Identity();

  return CallbackReturn::SUCCESS;
}

CallbackReturn OperationalSpaceController::on_configure(
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

  end_effector_frame_id = model_.getFrameId(end_effector_frame_);
  q = Eigen::VectorXd::Zero(model_.nq);
  dq = Eigen::VectorXd::Zero(model_.nv);
  tau = Eigen::VectorXd::Zero(model_.nv);
  J = Eigen::MatrixXd::Zero(6, model_.nv);

  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "Preparing the stiffness matrix and damping matrices...");

  setStiffnessAndDamping();

  RCLCPP_INFO(get_node()->get_logger(), "State interfaces set up.");

  return CallbackReturn::SUCCESS;
}

void OperationalSpaceController::setStiffnessAndDamping() {
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3)
      << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3)
      << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  damping.setZero();
  damping.topLeftCorner(3, 3)
      << 2.0 * sqrt(translational_stiffness) * Eigen::MatrixXd::Identity(3, 3);
  damping.bottomRightCorner(3, 3)
      << 2.0 * sqrt(rotational_stiffness) * Eigen::MatrixXd::Identity(3, 3);
}

Eigen::Matrix<double, 7, 1> OperationalSpaceController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>
        &tau_j_d) { // NOLINT (readability-identifier-naming)
  auto difference = tau_d_calculated - tau_j_d;
  auto tau_d_saturated =
      tau_j_d + difference.cwiseMax(delta_tau_max_).cwiseMin(-delta_tau_max_);
  return tau_d_saturated;
}

CallbackReturn OperationalSpaceController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  for (int i = 0; i < num_joints_; i++) {
    q[i] = state_interfaces_[i].get_value();
  }
  q_d_nullspace_ << q;

  pinocchio::forwardKinematics(model_, data_, q, dq);
  pinocchio::updateFramePlacements(model_, data_);

  end_effector_pose = data_.oMf[end_effector_frame_id];

  target_position_ = end_effector_pose.translation();
  target_orientation_ = Eigen::Quaterniond(end_effector_pose.rotation());
  target_pose_ =
      pinocchio::SE3(target_orientation_.toRotationMatrix(), target_position_);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OperationalSpaceController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

void OperationalSpaceController::target_pose_callback_(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  target_position_ << msg->pose.position.x, msg->pose.position.y,
      msg->pose.position.z;
  target_orientation_ =
      Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                         msg->pose.orientation.y, msg->pose.orientation.z);
}

} // namespace crisp_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(crisp_controllers::OperationalSpaceController,
                       controller_interface::ControllerInterface)
