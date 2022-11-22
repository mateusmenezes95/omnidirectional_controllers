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

#include <cmath>
#include <memory>

#include "omnidirectional_controllers/kinematics.hpp"

#include "omnidirectional_controllers/types.hpp"

namespace omnidirectional_controllers {

Kinematics::Kinematics(RobotParams robot_params)
  : robot_params_(robot_params) {
  this->initializeParams();
}

Kinematics::Kinematics() {
  this->initializeParams();
}

RobotVelocity Kinematics::getBodyVelocity(const std::vector<double> & wheels_vel) {
  RobotVelocity vel;
  double wm1 = wheels_vel.at(0);
  double wm2 = wheels_vel.at(1);
  double wm3 = wheels_vel.at(2);

  vel.vx = beta_ * (wm2 - wm3);
  vel.vy = alpha_ * (-wm1 + (0.5 * (wm2 + wm3)));
  vel.omega = ((1/robot_params_.robot_radius)*alpha_) * ((sin_gamma_ * wm1) + (0.5 * (wm2 + wm3)));

  vel.vx *= robot_params_.wheel_radius;
  vel.vy *= robot_params_.wheel_radius;
  vel.omega *= robot_params_.wheel_radius;

  return vel;
}

std::vector<double> Kinematics::getWheelsAngularVelocities(RobotVelocity vel) {
  double vx = vel.vx;
  double vy = vel.vy;
  double wl = vel.omega * robot_params_.robot_radius;

  angular_vel_vec_[0] = (-vy + wl) / robot_params_.wheel_radius;
  angular_vel_vec_[1] = ((vx * cos_gamma_) + (vy * sin_gamma_) + wl) / robot_params_.wheel_radius;
  angular_vel_vec_[2] = ((-vx * cos_gamma_) + (vy * sin_gamma_) + wl) / robot_params_.wheel_radius;

  return angular_vel_vec_;
}

void Kinematics::setRobotParams(RobotParams robot_params) {
  this->robot_params_ = robot_params;
  this->initializeParams();
}

void Kinematics::initializeParams() {
  angular_vel_vec_.reserve(OMNI_ROBOT_MAX_WHEELS);
  angular_vel_vec_ = {0, 0, 0, 0};
  cos_gamma_ = cos(robot_params_.gamma);
  sin_gamma_ = sin(robot_params_.gamma);
  alpha_ = 1 / (sin_gamma_ + 1);
  beta_ = 1 / (2*cos_gamma_);
}

Kinematics::~Kinematics() {}

}  // namespace omnidirectional_controllers
