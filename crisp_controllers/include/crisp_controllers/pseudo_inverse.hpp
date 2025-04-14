#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

namespace crisp_controllers {

inline Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd &matrix,
                                     double epsilon = 1e-4) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeFullU |
                                                    Eigen::ComputeFullV);
  Eigen::VectorXd singularValues = svd.singularValues();
  Eigen::VectorXd singularValuesInv(singularValues.size());
  for (int i = 0; i < singularValues.size(); ++i) {
    singularValuesInv[i] =
        singularValues[i] /
        (singularValues[i] * singularValues[i] + epsilon * epsilon);
  }

  return svd.matrixV() * singularValuesInv.asDiagonal() *
         svd.matrixU().transpose();
}

} // namespace crisp_controllers
