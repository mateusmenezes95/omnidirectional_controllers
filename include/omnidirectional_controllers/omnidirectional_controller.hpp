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

#ifndef OMNIDIRECTIONAL_CONTROLLERS__OMNIDIRECTIONAL_CONTROLLER_HPP_
#define OMNIDIRECTIONAL_CONTROLLERS__OMNIDIRECTIONAL_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "omnidirectional_controllers/kinematics.hpp"
#include "omnidirectional_controllers/odometry.hpp"
#include "omnidirectional_controllers/types.hpp"

namespace omnidirectional_controllers {
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class OmnidirectionalController : public controller_interface::ControllerInterface {
 public:
  OmnidirectionalController();
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  ~OmnidirectionalController();

 protected:
  struct WheelHandle {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_command;
  };

  std::vector<std::string> wheel_names_;
  std::vector<WheelHandle> registered_wheel_handles_;

  // Default parameters for axebot
  RobotParams robot_params_{0.1, 0.0505, 0.0};

  struct OdometryParams {
    bool open_loop = false;
    bool enable_odom_tf = true;
    std::string base_frame_id = "base_link";
    std::string odom_frame_id = "odom";
    std::string odom_numeric_integration_method = EULER_FORWARD;
    std::array<double, 6> pose_covariance_diagonal;
    std::array<double, 6> twist_covariance_diagonal;
  } odom_params_;

  bool use_stamped_vel_ = true;

  Kinematics omni_robot_kinematics_;
  Odometry odometry_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_ = nullptr;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr odometry_transform_publisher_ = nullptr;

  tf2_msgs::msg::TFMessage odometry_transform_message_;
  nav_msgs::msg::Odometry odometry_message_;

  // Timeout to consider cmd_vel commands old
  std::chrono::milliseconds cmd_vel_timeout_{500};

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_cmd_subscriber_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    vel_cmd_unstamped_subscriber_ = nullptr;

  rclcpp::Time previous_update_timestamp_{0};

  double publish_rate_{50.0};
  rclcpp::Duration publish_period_{0, 0};
  rclcpp::Time previous_publish_timestamp_{0};

 private:
  void velocityCommandStampedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr cmd_vel);
  void velocityCommandUnstampedCallback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel);
  geometry_msgs::msg::TwistStamped::SharedPtr cmd_vel_;
  double cos_gamma{0};
  double sin_gamma{0};

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
};

}  // namespace omnidirectional_controllers

#endif  // OMNIDIRECTIONAL_CONTROLLERS__OMNIDIRECTIONAL_CONTROLLER_HPP_
