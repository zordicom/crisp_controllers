#include <gtest/gtest.h>
#include <stdexcept>
#include "crisp_controllers/utils/fiters.hpp"

TEST(ExponentialMovingAverageTest, BasicFiltering) {
  double output = 0.0;
  double current = 1.0;
  double alpha = 0.5;
  
  double result = exponential_moving_average(output, current, alpha);
  EXPECT_DOUBLE_EQ(result, 0.5); // Should be (1-0.5)*1.0 + 0.5*0.0 = 0.5
}

TEST(ExponentialMovingAverageTest, AlphaLimits) {
  double output = 1.0;
  double current = 2.0;
  
  double result1 = exponential_moving_average(output, current, 0.01);
  EXPECT_NEAR(result1, 1.99, 1e-10);
  
  double result2 = exponential_moving_average(output, current, 0.99);
  EXPECT_NEAR(result2, 1.01, 1e-10);
}

TEST(ExponentialMovingAverageTest, TypeTest) {
  // Test with different numeric types
  {
    float output = 1.0f;
    float current = 2.0f;
    float result = exponential_moving_average(output, current, 0.5);
    EXPECT_FLOAT_EQ(result, 1.5f);
  }
  
  {
    int output = 1;
    int current = 2;
    int result = exponential_moving_average(output, current, 0.5);
    EXPECT_EQ(result, 1);
  }
}

TEST(ExponentialMovingAverageTest, Convergence) {
  double output = 0.0;
  double current = 1.0;
  double alpha = 0.5;
  
  for(int i = 0; i < 10; i++) {
    output = exponential_moving_average(output, current, alpha);
  }
  
  EXPECT_NEAR(output, current, 1e-3);
}

TEST(ExponentialMovingAverageTest, DeathTest) {
  double output = 0.0;
  double current = 1.0;
  
  EXPECT_THROW(exponential_moving_average(output, current, 0.0), std::invalid_argument);
  EXPECT_THROW(exponential_moving_average(output, current, 1.0), std::invalid_argument);
  EXPECT_THROW(exponential_moving_average(output, current, -0.5), std::invalid_argument);
  EXPECT_THROW(exponential_moving_average(output, current, 1.5), std::invalid_argument);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

