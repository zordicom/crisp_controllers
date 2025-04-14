#include <crisp_controllers/friction_model.hpp>
#include <crisp_controllers/joint_limits.hpp>
#include <crisp_controllers/cartesian_impedance_controller.hpp>
#include <crisp_controllers/pseudo_inverse.hpp>

#include <pinocchio/algorithm/frames.hxx>
#include <binders.h>
#include <cassert>
#include <cmath>
#include <memory>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>

using namespace std::chrono_literals;

typedef Eigen::Matrix<double, 6, 1> Vector6d;

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
  // The first num_joints_ are the joint positions, the next num_joints_ are the
  // joint velocities, and the last num_joints_ are the joint torques
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
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

  size_t num_joints = params_.joints.size();
  for (size_t i = 0; i < num_joints; i++) {
    q[i] = state_interfaces_[i].get_value();
    dq[i] = state_interfaces_[num_joints + i].get_value();
    tau[i] = state_interfaces_[2 * num_joints + i].get_value();
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

  auto max_delta_ =
      Eigen::Map<Eigen::VectorXd>(params_.max_delta.data(), num_joints);
  error.cwiseMin(max_delta_).cwiseMax(-max_delta_);

  J.setZero();
  pinocchio::computeFrameJacobian(model_, data_, q, end_effector_frame_id,
                                  pinocchio::ReferenceFrame::LOCAL, J);

  Eigen::MatrixXd J_pinv(model_.nv, 6);
  Eigen::MatrixXd Id_nv(model_.nv, model_.nv);

  J_pinv = pseudoInverse(J);
  Eigen::MatrixXd nullspace_projection = Id_nv - J_pinv * J;

  // Now we compute all terms of the control law
  Eigen::VectorXd tau_task(model_.nv), tau_d(model_.nv),
      tau_secondary(model_.nv), tau_nullspace(model_.nv),
      tau_friction(model_.nv), tau_coriolis(model_.nv),
      tau_joint_limits(model_.nv);

  tau_task << J.transpose() * (stiffness * error - damping * (J * dq));

  tau_joint_limits = get_joint_limit_torque(q, model_.lowerPositionLimit,
                                            model_.upperPositionLimit);

  tau_secondary << params_.nullspace_stiffness * (q_ref - q) -
                       2 * sqrt(params_.nullspace_stiffness) * dq;
  tau_nullspace << nullspace_projection * tau_secondary;

  tau_friction = params_.use_friction ? get_friction(dq)
                                      : Eigen::VectorXd::Zero(model_.nv);

  if (params_.use_coriolis_compensation) {
    pinocchio::computeAllTerms(model_, data_, q, dq);
    tau_coriolis = pinocchio::computeCoriolisMatrix(model_, data_, q, dq) * dq;
  } else {
    tau_coriolis = Eigen::VectorXd::Zero(model_.nv);
  }

  tau_d << tau_task + tau_nullspace + tau_friction + tau_coriolis +
               tau_joint_limits;

  for (size_t i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_d[i]);
  }

  // TODO: Updat parameters online
  /*setStiffnessAndDamping();*/

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
  J = Eigen::MatrixXd::Zero(6, model_.nv);

  setStiffnessAndDamping();

  RCLCPP_INFO(get_node()->get_logger(), "State interfaces set up.");

  return CallbackReturn::SUCCESS;
}

void CartesianImpedanceController::setStiffnessAndDamping() {
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3)
      << params_.translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3)
      << params_.rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  damping.setZero();
  damping.topLeftCorner(3, 3)
      << 2.0 * sqrt(params_.translational_stiffness) * Eigen::MatrixXd::Identity(3, 3);
  damping.bottomRightCorner(3, 3)
      << 2.0 * sqrt(params_.rotational_stiffness) * Eigen::MatrixXd::Identity(3, 3);
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
