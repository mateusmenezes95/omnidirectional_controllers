// MIT License

// Copyright (c) 2022 Mateus Menezes

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <gmock/gmock.h>

#include <vector>

#include "omnidirectional_controllers/kinematics.hpp"
#include "omnidirectional_controllers/types.hpp"

constexpr double TOLERANCE = 1e-3;

class TestOmnidirectionalControllersKinematics : public ::testing::Test {
 protected:
  omnidirectional_controllers::Kinematics kinematics_;
};

TEST_F(TestOmnidirectionalControllersKinematics, TestForwardKinematics) {
  std::vector<double> angular_velocity;
  angular_velocity.reserve(3);
  kinematics_.setRobotParams({0.053112205, 0.1, DEG2RAD(30)});

  omnidirectional_controllers::RobotVelocity vel;

  angular_velocity = {0.0, 1.63056, -1.63056};
  vel = kinematics_.getBodyVelocity(angular_velocity);

  EXPECT_NEAR(vel.vx, 0.1, TOLERANCE);
  EXPECT_NEAR(vel.vy, 0, TOLERANCE);
  EXPECT_NEAR(vel.omega, 0, TOLERANCE);

  angular_velocity = {-1.88281, 0.94140, 0.94140};
  vel = kinematics_.getBodyVelocity(angular_velocity);

  EXPECT_NEAR(vel.vx, 0, TOLERANCE);
  EXPECT_NEAR(vel.vy, 0.1, TOLERANCE);
  EXPECT_NEAR(vel.omega, 0, TOLERANCE);

  angular_velocity = {0.18828, 0.18828, 0.18828};
  vel = kinematics_.getBodyVelocity(angular_velocity);

  EXPECT_NEAR(vel.vx, 0, TOLERANCE);
  EXPECT_NEAR(vel.vy, 0, TOLERANCE);
  EXPECT_NEAR(vel.omega, 0.1, TOLERANCE);

  angular_velocity = {-1.88281, 2.57196, -0.68915};
  vel = kinematics_.getBodyVelocity(angular_velocity);

  EXPECT_NEAR(vel.vx, 0.1, TOLERANCE);
  EXPECT_NEAR(vel.vy, 0.1, TOLERANCE);
  EXPECT_NEAR(vel.omega, 0, TOLERANCE);
}

TEST_F(TestOmnidirectionalControllersKinematics, TestInverseKinematics) {
  std::vector<double> angular_velocities;
  omnidirectional_controllers::RobotVelocity vel;
  kinematics_.setRobotParams({0.053112205, 0.1, DEG2RAD(30)});

  vel = {0.1, 0, 0};
  angular_velocities = kinematics_.getWheelsAngularVelocities(vel);

  ASSERT_GT(angular_velocities.size(), 0.0);
  EXPECT_NEAR(angular_velocities[0], 0.0, TOLERANCE);
  EXPECT_NEAR(angular_velocities[1], 1.63056, TOLERANCE);
  EXPECT_NEAR(angular_velocities[2], -1.63056, TOLERANCE);

  vel = {0, 0.1, 0};
  angular_velocities = kinematics_.getWheelsAngularVelocities(vel);

  ASSERT_GT(angular_velocities.size(), 0.0);
  EXPECT_NEAR(angular_velocities[0], -1.88281, TOLERANCE);
  EXPECT_NEAR(angular_velocities[1], 0.94140, TOLERANCE);
  EXPECT_NEAR(angular_velocities[2], 0.94140, TOLERANCE);

  vel = {0, 0, 0.1};
  angular_velocities = kinematics_.getWheelsAngularVelocities(vel);

  ASSERT_GT(angular_velocities.size(), 0.0);
  EXPECT_NEAR(angular_velocities[0], 0.18828, TOLERANCE);
  EXPECT_NEAR(angular_velocities[1], 0.18828, TOLERANCE);
  EXPECT_NEAR(angular_velocities[2], 0.18828, TOLERANCE);

  vel = {0.1, 0.1, 0};
  angular_velocities = kinematics_.getWheelsAngularVelocities(vel);

  ASSERT_GT(angular_velocities.size(), 0.0);
  EXPECT_NEAR(angular_velocities[0], -1.88281, TOLERANCE);
  EXPECT_NEAR(angular_velocities[1], 2.57196, TOLERANCE);
  EXPECT_NEAR(angular_velocities[2], -0.68915, TOLERANCE);
}
