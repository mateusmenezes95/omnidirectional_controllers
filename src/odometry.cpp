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
#include <exception>
#include <limits>
#include <sstream>
#include <string>

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
  if (params.wheel_radius < std::numeric_limits<double>::epsilon()) {
    std::stringstream error;
    error << "Invalid wheel radius " << params.wheel_radius << " set!" << std::endl;
    throw std::runtime_error(error.str());
  }

  if (params.robot_radius < std::numeric_limits<double>::epsilon()) {
    std::stringstream error;
    error << "Invalid robot radius " << params.wheel_radius << " set!" << std::endl;
    throw std::runtime_error(error.str());
  }

  this->robot_params_ = params;
  robot_kinematics_.setRobotParams(robot_params_);
  is_robot_param_set_ = true;
}

void Odometry::updateOpenLoop(RobotVelocity vel, double dt_) {
  this->body_vel_ = vel;
  this->dt_ = dt_;
  this->integrateVelocities();
}

void Odometry::update(const std::vector<double> & wheels_vel, double dt) {
  if (!is_robot_param_set_) {
    throw std::runtime_error(std::string("Robot parameters was not set or not set properly!"));
  }
  this->dt_ = dt;
  this->body_vel_ = robot_kinematics_.getBodyVelocity(wheels_vel);
  this->integrateVelocities();
}

RobotPose Odometry::getPose() const {
  return pose_;
}

RobotVelocity Odometry::getBodyVelocity() const {
  return body_vel_;
}

void Odometry::reset() {
  pose_ = {0, 0, 0};
}

void Odometry::integrateByRungeKutta() {
  double theta_bar = pose_.theta + (body_vel_.omega*dt_ / 2);
  pose_.x = pose_.x + (body_vel_.vx * cos(theta_bar) - body_vel_.vy * sin(theta_bar)) * dt_;
  pose_.y = pose_.y + (body_vel_.vx * sin(theta_bar) + body_vel_.vy * cos(theta_bar)) * dt_;
  pose_.theta = pose_.theta + body_vel_.omega*dt_;
}

void Odometry::integrateByEuler() {
  pose_.x = pose_.x + (body_vel_.vx * cos(pose_.theta) - body_vel_.vy * sin(pose_.theta)) * dt_;
  pose_.y = pose_.y + (body_vel_.vx * sin(pose_.theta) + body_vel_.vy * cos(pose_.theta)) * dt_;
  pose_.theta = pose_.theta + body_vel_.omega*dt_;
}

void Odometry::integrateVelocities() {
  if (numeric_integration_method_ == RUNGE_KUTTA2) {
    this->integrateByRungeKutta();
    return;
  }
  // Euler method is the odometry class default!
  this->integrateByEuler();
  this->dt_ = 0;
  this->body_vel_ = {0, 0, 0};
}

Odometry::~Odometry() {}

}  // namespace omnidirectional_controllers
