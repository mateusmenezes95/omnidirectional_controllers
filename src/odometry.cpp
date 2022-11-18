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

#include "omnidirectional_controllers/odometry.hpp"

#include <cmath>

#include "omnidirectional_controllers/types.hpp"

namespace omnidirectional_controllers {

Odometry::Odometry() {}

bool Odometry::setNumericIntegrationMethod(const std::string & numeric_integration_method) {
  if (numeric_integration_method != EULER_FORWARD &&
      numeric_integration_method != RUNGE_KUTTA2) {
      this->numeric_integration_method_ = EULER_FORWARD;
      return false;
  }
  this->numeric_integration_method_ = numeric_integration_method;
  return true;
}

void Odometry::setRobotParams(RobotParams params) {
  robot_params_ = params;
}

void Odometry::updateOpenLoop(RobotVelocity vel, double dt) {
  if (numeric_integration_method_ == RUNGE_KUTTA2) {
    this->integrateByRungeKutta(vel, dt);
    return;
  }
  // Euler method is the odometry class default!
  this->integrateByEuler(vel, dt);
}

RobotPose Odometry::getPose() const {
  return pose_;
}

void Odometry::reset() {
  pose_ = {0, 0, 0};
}

void Odometry::integrateByRungeKutta(RobotVelocity vel, double dt) {
  double theta_bar = pose_.theta + (vel.omega*dt / 2);
  pose_.x = pose_.x + (vel.vx * cos(theta_bar) + vel.vy * sin(theta_bar)) * dt;
  pose_.y = pose_.y + (-vel.vx * sin(theta_bar) + vel.vy * cos(theta_bar)) * dt;
  pose_.theta = pose_.theta + vel.omega*dt;
}

void Odometry::integrateByEuler(RobotVelocity vel, double dt) {
  pose_.x = pose_.x + (vel.vx * cos(pose_.theta) + vel.vy * sin(pose_.theta)) * dt;
  pose_.y = pose_.y + (-vel.vx * sin(pose_.theta) + vel.vy * cos(pose_.theta)) * dt;
  pose_.theta = pose_.theta + vel.omega*dt;
}

Odometry::~Odometry() {}

}  // namespace omnidirectional_controllers
