// Explicit template instantiation for Pinocchio templates
// This file instantiates the most commonly used Pinocchio templates once
// to avoid redundant instantiation in multiple translation units

#include <Eigen/Core>
#include <Eigen/Dense>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/motion.hpp>
#include <pinocchio/spatial/force.hpp>
#include <pinocchio/spatial/inertia.hpp>

namespace pinocchio {

// Explicitly instantiate the core Model and Data templates
template class ModelTpl<double>;
template class DataTpl<double>;

// Explicitly instantiate spatial types
template class SE3Tpl<double>;
template class MotionTpl<double>;
template class ForceTpl<double>;
template class InertiaTpl<double>;

// Force instantiation of commonly used algorithm functions
// by creating dummy functions that use them
namespace {

void force_instantiations() {
    // This function is never called, but forces template instantiation
    ModelTpl<double> model;
    DataTpl<double> data(model);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);

    // Force instantiation of forwardKinematics variants
    forwardKinematics(model, data, q);
    forwardKinematics(model, data, q, v);
    forwardKinematics(model, data, q, v, a);

    // Force instantiation of updateFramePlacements
    updateFramePlacements(model, data);

    // Force instantiation of computeJointJacobians
    computeJointJacobians(model, data, q);
    computeJointJacobians(model, data);

    // Force instantiation of frame operations
    if (model.nframes > 0) {
        FrameIndex frame_id = 0;
        getFrameVelocity(model, data, frame_id, LOCAL_WORLD_ALIGNED);
        getFrameAcceleration(model, data, frame_id, LOCAL_WORLD_ALIGNED);

        Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model.nv);
        getFrameJacobian(model, data, frame_id, LOCAL_WORLD_ALIGNED, J);
    }

    // Force instantiation of dynamics algorithms
    computeAllTerms(model, data, q, v);
    aba(model, data, q, v, tau, Convention::WORLD);
    rnea(model, data, q, v, a);

    // Force instantiation of neutral configuration
    neutral(model, q);
}

} // anonymous namespace

} // namespace pinocchio