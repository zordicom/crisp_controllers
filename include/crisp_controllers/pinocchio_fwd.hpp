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

} // namespace pinocchio

#endif // CRISP_CONTROLLERS_PINOCCHIO_FWD_HPP