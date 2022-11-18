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

#include <cmath>

#include "omnidirectional_controllers/odometry.hpp"
#include "omnidirectional_controllers/types.hpp"

constexpr double TOLERANCE = 1e-6;

class TestOmnidirectionalControllersOdometry : public ::testing::Test {
 protected:
  omnidirectional_controllers::Odometry odom_;

  void testMethodOnlyWithLinearVelocities(std::string method_name) {
    ASSERT_TRUE(odom_.setNumericIntegrationMethod(method_name));

    double time = 0;
    constexpr double dt = 0.1;

    omnidirectional_controllers::RobotVelocity vel = {1.0, 1.0, 0.0};

    for (size_t i = 0; i < 10; i++) {
      time += dt;
      odom_.updateOpenLoop(vel, dt);
    }

    EXPECT_NEAR(time, 1.0, TOLERANCE);
    EXPECT_NEAR(odom_.getPose().x, 1.0, TOLERANCE);
    EXPECT_NEAR(odom_.getPose().y, 1.0, TOLERANCE);
    EXPECT_NEAR(odom_.getPose().theta, 0, TOLERANCE);

    odom_.reset();

    EXPECT_NEAR(odom_.getPose().x, 0, TOLERANCE);
    EXPECT_NEAR(odom_.getPose().y, 0, TOLERANCE);
    EXPECT_NEAR(odom_.getPose().theta, 0, TOLERANCE);
  }
};

TEST_F(TestOmnidirectionalControllersOdometry, TestIfNumericIntegrationMethodIsSetProperly) {
  ASSERT_TRUE(odom_.setNumericIntegrationMethod("euler_forward"));
  ASSERT_TRUE(odom_.setNumericIntegrationMethod("runge_kutta2"));
  ASSERT_FALSE(odom_.setNumericIntegrationMethod("other_method"));
}

TEST_F(TestOmnidirectionalControllersOdometry, TestEulerMethodOnlyWithLinearVelocities) {
  this->testMethodOnlyWithLinearVelocities("euler_forward");
}

TEST_F(TestOmnidirectionalControllersOdometry, TestEulerMethodWithLinearAndAngularVelocities) {
  ASSERT_TRUE(odom_.setNumericIntegrationMethod("euler_forward"));

  constexpr double dt = 0.1;
  omnidirectional_controllers::RobotVelocity vel = {1.0, 1.0, 1.0};

  odom_.updateOpenLoop(vel, dt);
  EXPECT_NEAR(odom_.getPose().x, 0.1, TOLERANCE);
  EXPECT_NEAR(odom_.getPose().y, 0.1, TOLERANCE);
  EXPECT_NEAR(odom_.getPose().theta, 0.1, TOLERANCE);

  odom_.updateOpenLoop(vel, dt);
  EXPECT_NEAR(odom_.getPose().x, 0.209484, TOLERANCE);
  EXPECT_NEAR(odom_.getPose().y, 0.189517, TOLERANCE);
  EXPECT_NEAR(odom_.getPose().theta, 0.2, TOLERANCE);

  odom_.updateOpenLoop(vel, dt);
  EXPECT_NEAR(odom_.getPose().x, 0.327358, TOLERANCE);
  EXPECT_NEAR(odom_.getPose().y, 0.267657, TOLERANCE);
  EXPECT_NEAR(odom_.getPose().theta, 0.3, TOLERANCE);

  odom_.reset();

  EXPECT_NEAR(odom_.getPose().x, 0, TOLERANCE);
  EXPECT_NEAR(odom_.getPose().y, 0, TOLERANCE);
  EXPECT_NEAR(odom_.getPose().theta, 0, TOLERANCE);
}

TEST_F(TestOmnidirectionalControllersOdometry, TestRungeKuttaMethodOnlyWithLinearVelocities) {
  this->testMethodOnlyWithLinearVelocities("runge_kutta2");
}

TEST_F(TestOmnidirectionalControllersOdometry, TestRungeKuttaMethodWithLinearAndAngularVelocities) {
  ASSERT_TRUE(odom_.setNumericIntegrationMethod("runge_kutta2"));

  constexpr double dt = 0.1;
  omnidirectional_controllers::RobotVelocity vel = {1.0, 1.0, 1.0};

  odom_.updateOpenLoop(vel, dt);
  EXPECT_NEAR(odom_.getPose().x, 0.104873, TOLERANCE);
  EXPECT_NEAR(odom_.getPose().y, 0.094877, TOLERANCE);
  EXPECT_NEAR(odom_.getPose().theta, 0.1, TOLERANCE);

  odom_.updateOpenLoop(vel, dt);
  EXPECT_NEAR(odom_.getPose().x, 0.218694, TOLERANCE);
  EXPECT_NEAR(odom_.getPose().y, 0.178810, TOLERANCE);
  EXPECT_NEAR(odom_.getPose().theta, 0.2, TOLERANCE);

  odom_.updateOpenLoop(vel, dt);
  EXPECT_NEAR(odom_.getPose().x, 0.340326, TOLERANCE);
  EXPECT_NEAR(odom_.getPose().y, 0.250961, TOLERANCE);
  EXPECT_NEAR(odom_.getPose().theta, 0.3, TOLERANCE);

  odom_.reset();

  EXPECT_NEAR(odom_.getPose().x, 0, TOLERANCE);
  EXPECT_NEAR(odom_.getPose().y, 0, TOLERANCE);
  EXPECT_NEAR(odom_.getPose().theta, 0, TOLERANCE);
}
