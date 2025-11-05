#ifndef CRISP_CONTROLLERS_PINOCCHIO_FWD_HPP
#define CRISP_CONTROLLERS_PINOCCHIO_FWD_HPP

// Forward declarations and extern template declarations for Pinocchio
// Include this header instead of pinocchio headers directly to avoid
// redundant template instantiation

#include <pinocchio/fwd.hpp>
#include <Eigen/Core>

// Tell the compiler these templates are instantiated elsewhere
namespace pinocchio {

// Extern template declarations - prevents implicit instantiation
extern template class ModelTpl<double>;
extern template class DataTpl<double>;
extern template class SE3Tpl<double>;
extern template class MotionTpl<double>;
extern template class ForceTpl<double>;
extern template class InertiaTpl<double>;

// Algorithm functions
extern template void forwardKinematics<double, 0, JointCollectionDefaultTpl>(
    const ModelTpl<double>&,
    DataTpl<double>&,
    const Eigen::MatrixBase<Eigen::VectorXd>&);

extern template void forwardKinematics<double, 0, JointCollectionDefaultTpl>(
    const ModelTpl<double>&,
    DataTpl<double>&,
    const Eigen::MatrixBase<Eigen::VectorXd>&,
    const Eigen::MatrixBase<Eigen::VectorXd>&);

extern template void updateFramePlacements<double, 0, JointCollectionDefaultTpl>(
    const ModelTpl<double>&,
    DataTpl<double>&);

extern template void computeJointJacobians<double, 0, JointCollectionDefaultTpl>(
    const ModelTpl<double>&,
    DataTpl<double>&,
    const Eigen::MatrixBase<Eigen::VectorXd>&);

extern template void getFrameJacobian<double, 0, JointCollectionDefaultTpl>(
    const ModelTpl<double>&,
    DataTpl<double>&,
    const FrameIndex,
    const ReferenceFrame,
    const Eigen::MatrixBase<Eigen::Matrix<double, 6, Eigen::Dynamic>>&);

extern template MotionTpl<double> getFrameVelocity<double, 0, JointCollectionDefaultTpl>(
    const ModelTpl<double>&,
    const DataTpl<double>&,
    const FrameIndex,
    const ReferenceFrame);

extern template void computeAllTerms<double, 0, JointCollectionDefaultTpl>(
    const ModelTpl<double>&,
    DataTpl<double>&,
    const Eigen::MatrixBase<Eigen::VectorXd>&,
    const Eigen::MatrixBase<Eigen::VectorXd>&);

extern template const Eigen::VectorXd& aba<double, 0, JointCollectionDefaultTpl>(
    const ModelTpl<double>&,
    DataTpl<double>&,
    const Eigen::MatrixBase<Eigen::VectorXd>&,
    const Eigen::MatrixBase<Eigen::VectorXd>&,
    const Eigen::MatrixBase<Eigen::VectorXd>&);

extern template const Eigen::VectorXd& rnea<double, 0, JointCollectionDefaultTpl>(
    const ModelTpl<double>&,
    DataTpl<double>&,
    const Eigen::MatrixBase<Eigen::VectorXd>&,
    const Eigen::MatrixBase<Eigen::VectorXd>&,
    const Eigen::MatrixBase<Eigen::VectorXd>&);

} // namespace pinocchio

#endif // CRISP_CONTROLLERS_PINOCCHIO_FWD_HPP