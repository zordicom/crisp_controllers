#include <Eigen/Dense>

using namespace Eigen;

inline Eigen::VectorXd get_joint_limit_torque(const VectorXd &joint_positions,
                                              const VectorXd &lower_limits,
                                              const VectorXd &upper_limits,
                                              double safe_range,  // [rad]
                                              double max_torque   // [Nm]
) {

  VectorXd torques = VectorXd::Zero(joint_positions.size());

  VectorXd dist_to_lower = joint_positions - lower_limits;
  VectorXd dist_to_upper = upper_limits - joint_positions;

  Array<bool, Dynamic, 1> near_lower = (dist_to_lower.array() < safe_range);
  Array<bool, Dynamic, 1> near_upper = (dist_to_upper.array() < safe_range);

  ArrayXd lower_ratios = ((safe_range - dist_to_lower.array()) / safe_range)
                             .cwiseMax(0.0)
                             .cwiseMin(1.0);
  ArrayXd upper_ratios = ((safe_range - dist_to_upper.array()) / safe_range)
                             .cwiseMax(0.0)
                             .cwiseMin(1.0);

  torques = (near_lower.select(max_torque * lower_ratios, 0.0) -
             near_upper.select(max_torque * upper_ratios, 0.0))
                .matrix();
  return torques;
}
