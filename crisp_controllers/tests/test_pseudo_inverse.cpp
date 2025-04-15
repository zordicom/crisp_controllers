#include <gtest/gtest.h>
#include "crisp_controllers/utils/pseudo_inverse.hpp"

TEST(PseudoInverseTest, SquareMatrixTest) {
  // Create a simple 2x2 matrix
  Eigen::MatrixXd A(2, 2);
  A << 4, 7,
       2, 6;
  
  Eigen::MatrixXd A_pinv = crisp_controllers::pseudoInverse(A);
  
  // Test if A * A_pinv * A ≈ A
  Eigen::MatrixXd result = A * A_pinv * A;
  EXPECT_TRUE(result.isApprox(A, 1e-10));
}

TEST(PseudoInverseTest, RectangularMatrixTest) {
  // Create a 3x2 matrix
  Eigen::MatrixXd B(3, 2);
  B << 1, 2,
       3, 4,
       5, 6;
  
  Eigen::MatrixXd B_pinv = crisp_controllers::pseudoInverse(B);
  
  // Test if B * B_pinv * B ≈ B
  Eigen::MatrixXd result = B * B_pinv * B;
  EXPECT_TRUE(result.isApprox(B, 1e-10));
}

TEST(PseudoInverseTest, MoorePenroseTest) {
  // Create a 2x3 matrix
  Eigen::MatrixXd C(2, 3);
  C << 1, 2, 3,
       4, 5, 6;
  
  Eigen::MatrixXd C_pinv = crisp_controllers::pseudoInverseMoorePenrose(C);
  
  // Test Moore-Penrose conditions
  // 1. C * C_pinv * C = C
  EXPECT_TRUE((C * C_pinv * C).isApprox(C, 1e-10));
  
  // 2. C_pinv * C * C_pinv = C_pinv
  EXPECT_TRUE((C_pinv * C * C_pinv).isApprox(C_pinv, 1e-10));
  
  // 3. (C * C_pinv)' = C * C_pinv
  EXPECT_TRUE((C * C_pinv).transpose().isApprox(C * C_pinv, 1e-10));
  
  // 4. (C_pinv * C)' = C_pinv * C
  EXPECT_TRUE((C_pinv * C).transpose().isApprox(C_pinv * C, 1e-10));
}

TEST(PseudoInverseTest, ZeroMatrixTest) {
  // Test with near-zero matrix
  Eigen::MatrixXd D = Eigen::MatrixXd::Zero(2, 2);
  
  Eigen::MatrixXd D_pinv1 = crisp_controllers::pseudoInverse(D);
  Eigen::MatrixXd D_pinv2 = crisp_controllers::pseudoInverseMoorePenrose(D);
  
  // Both should return zero matrix
  EXPECT_TRUE(D_pinv1.isApprox(Eigen::MatrixXd::Zero(2, 2), 1e-10));
  EXPECT_TRUE(D_pinv2.isApprox(Eigen::MatrixXd::Zero(2, 2), 1e-10));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

