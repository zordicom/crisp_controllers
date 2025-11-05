// Explicit template instantiation for Pinocchio templates
// This file instantiates all commonly used Pinocchio templates once
// to avoid redundant instantiation in multiple translation units

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/motion.hpp>
#include <pinocchio/spatial/force.hpp>
#include <pinocchio/spatial/inertia.hpp>
#include <pinocchio/spatial/explog.hpp>

namespace pinocchio {

// Explicitly instantiate Model and Data templates
template class ModelTpl<double>;
template class DataTpl<double>;

// Explicitly instantiate spatial types
template class SE3Tpl<double>;
template class MotionTpl<double>;
template class ForceTpl<double>;
template class InertiaTpl<double>;

// Explicitly instantiate algorithm functions for double precision
// Note: These are function templates, so we need to instantiate them with specific calls

// Forward kinematics
template void forwardKinematics<double, 0, JointCollectionDefaultTpl>(
    const ModelTpl<double>&,
    DataTpl<double>&,
    const Eigen::MatrixBase<Eigen::VectorXd>&);

template void forwardKinematics<double, 0, JointCollectionDefaultTpl>(
    const ModelTpl<double>&,
    DataTpl<double>&,
    const Eigen::MatrixBase<Eigen::VectorXd>&,
    const Eigen::MatrixBase<Eigen::VectorXd>&);

template void forwardKinematics<double, 0, JointCollectionDefaultTpl>(
    const ModelTpl<double>&,
    DataTpl<double>&,
    const Eigen::MatrixBase<Eigen::VectorXd>&,
    const Eigen::MatrixBase<Eigen::VectorXd>&,
    const Eigen::MatrixBase<Eigen::VectorXd>&);

// Update frame placements
template void updateFramePlacements<double, 0, JointCollectionDefaultTpl>(
    const ModelTpl<double>&,
    DataTpl<double>&);

// Compute joint Jacobians
template void computeJointJacobians<double, 0, JointCollectionDefaultTpl>(
    const ModelTpl<double>&,
    DataTpl<double>&,
    const Eigen::MatrixBase<Eigen::VectorXd>&);

template void computeJointJacobians<double, 0, JointCollectionDefaultTpl>(
    const ModelTpl<double>&,
    DataTpl<double>&);

// Get frame Jacobian
template void getFrameJacobian<double, 0, JointCollectionDefaultTpl>(
    const ModelTpl<double>&,
    DataTpl<double>&,
    const FrameIndex,
    const ReferenceFrame,
    const Eigen::MatrixBase<Eigen::Matrix<double, 6, Eigen::Dynamic>>&);

// Get frame velocity
template MotionTpl<double> getFrameVelocity<double, 0, JointCollectionDefaultTpl>(
    const ModelTpl<double>&,
    const DataTpl<double>&,
    const FrameIndex,
    const ReferenceFrame);

// Get frame acceleration
template MotionTpl<double> getFrameAcceleration<double, 0, JointCollectionDefaultTpl>(
    const ModelTpl<double>&,
    const DataTpl<double>&,
    const FrameIndex,
    const ReferenceFrame);

// Compute all terms (for dynamics)
template void computeAllTerms<double, 0, JointCollectionDefaultTpl>(
    const ModelTpl<double>&,
    DataTpl<double>&,
    const Eigen::MatrixBase<Eigen::VectorXd>&,
    const Eigen::MatrixBase<Eigen::VectorXd>&);

// ABA (Articulated Body Algorithm)
template const Eigen::VectorXd& aba<double, 0, JointCollectionDefaultTpl>(
    const ModelTpl<double>&,
    DataTpl<double>&,
    const Eigen::MatrixBase<Eigen::VectorXd>&,
    const Eigen::MatrixBase<Eigen::VectorXd>&,
    const Eigen::MatrixBase<Eigen::VectorXd>&);

// RNEA (Recursive Newton-Euler Algorithm)
template const Eigen::VectorXd& rnea<double, 0, JointCollectionDefaultTpl>(
    const ModelTpl<double>&,
    DataTpl<double>&,
    const Eigen::MatrixBase<Eigen::VectorXd>&,
    const Eigen::MatrixBase<Eigen::VectorXd>&,
    const Eigen::MatrixBase<Eigen::VectorXd>&);

// Neutral configuration
template void neutral<double, 0, JointCollectionDefaultTpl>(
    const ModelTpl<double>&,
    const Eigen::MatrixBase<Eigen::VectorXd>&);

} // namespace pinocchio