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

#ifndef OMNIDIRECTIONAL_CONTROLLERS__ODOMETRY_HPP_
#define OMNIDIRECTIONAL_CONTROLLERS__ODOMETRY_HPP_

#include <string>
#include <vector>

#include "omnidirectional_controllers/kinematics.hpp"
#include "omnidirectional_controllers/types.hpp"

namespace omnidirectional_controllers {

constexpr char EULER_FORWARD[] = "euler_forward";
constexpr char RUNGE_KUTTA2[] = "runge_kutta2";

class Odometry {
 public:
  Odometry();
  ~Odometry();
  bool setNumericIntegrationMethod(const std::string & numeric_integration_method);
  void setRobotParams(RobotParams params);
  void updateOpenLoop(RobotVelocity vel, double dt);
  void update(const std::vector<double> & wheels_vel, double dt);
  RobotPose getPose() const;
  RobotVelocity getBodyVelocity() const;
  void reset();

 protected:
  void integrateByRungeKutta();
  void integrateByEuler();
  void integrateVelocities();
  RobotVelocity body_vel_{0, 0, 0};
  double dt_;

 private:
  RobotPose pose_{0, 0, 0};
  RobotParams robot_params_{0, 0, 0};
  std::string numeric_integration_method_ = EULER_FORWARD;
  Kinematics robot_kinematics_;
  bool is_robot_param_set_{false};
};

}  // namespace omnidirectional_controllers

#endif  // OMNIDIRECTIONAL_CONTROLLERS__ODOMETRY_HPP_
