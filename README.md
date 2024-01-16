# Omnidirectional Controllers

This package provides ROS 2 controllers for controlling Omnidirectional robots with three wheels. It is based on the concepts of [ros2_control] and [ros2_controllers]. Initially, only forward and inverse kinematics, based solely on the [diff_drive_controller], and odometry calculation have been implemented. The input for control is robot body velocity ($\dot{x}$, $\dot{y}$, $\dot{\theta}$) commands, which are translated into wheel commands ($\omega_1$, $\omega_2$, $\omega_3$) for an omnidirectional robot with three wheels. Odometry is computed from hardware feedback and published. It is worth noting that there are plans to further develop advanced linear and non-linear controllers, such as Model Predictive Control (MPC).

See the documentation [Omnidirectional Robot Kinematics and Odometry](doc/kinematics_and_odometry.md) for more details.

**Author:** Mateus Menezes<br />
**Maintainer:** Mateus Menezes, mateusmenezes95@gmail.com

## Build status

ROS2 Distro | Branch | Build status |
:---------: | :----: | :----------: |
**Humble** | [`humble`](https://github.com/mateusmenezes95/omnidirectional_controllers/tree/humble) | [![Build From Source](https://github.com/mateusmenezes95/omnidirectional_controllers/actions/workflows/humble-source-build.yaml/badge.svg)](https://github.com/mateusmenezes95/omnidirectional_controllers/actions/workflows/humble-source-build.yaml)

## Installation Premises

1. This repository has been tested on [ROS2 Humble] and with [Classic Gazebo 11];

2. These instructions assume that you have already installed ROS2 Humble Hawskbill on your machine. If not, please follow the recommended [recommended ubuntu installation tutorial];

3. Before installing the package, you will need to have an ament workspace set up. If you don't have one, follow the instructions in the [Creating a workspace tutorial]. Once you have created the workspace, clone this repository in the source folder of your workspace.

## Installation

> **ATTENTION:** These commands assume that you have created a workspace called "ros_ws" in your home folder. If you used a different directory or name, please adjust the commands accordingly.

After installing ROS2 and creating the workspace, clone this repository in your workspace:

```
cd ~/ros_ws/src
git clone https://github.com/mateusmenezes95/omnidirectional_controllers
```

Install the binary dependencies by running the following command in the root of your workspace:

```
cd ~/ros_ws
rosdep init
rosdep update
sudo apt update
rosdep install --from-paths src/omnidirectional_controllers --ignore-src -r -y --rosdistro humble
```

If all dependencies are already installed, you should see the message "All required rosdeps installed successfully."

## Building

Run the following command to build the package:

```
cd ~/ros_ws
colcon build --symlink-install --event-handlers console_direct+
```

> Run `colcon build --help` to understand the arguments passed!
> If you want to generate the compile_commands.json file, add the argument `--cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1` to the command above.

After building the package, open a new terminal and navigate to your workspace. Then, source the overlay by running the following command:

```
source /opt/ros/foxy/setup.bash
cd ~/ros_ws
. install/local_setup.bash
```

> See [Source the overlay] to learn about underlay and overlay concepts.

## Usage

You must follow the three steps explained in [Running the Framework for Your Robot](https://control.ros.org/master/doc/getting_started/getting_started.html#running-the-framework-for-your-robot) tutorial.

For an concrete example of how to use the Omnidirectional controllers, refer to the Axebot simulation's [controller configuration], [ros2_control URDF], and [launch file].

### Subscribed Topic

* **`/omnidirectional_controller/cmd_vel_unstamped`** ([geometry_msgs/msg/Twist])

    Velocity twist from which the controller extracts the x and y component of the linear velocity and the z component of the angular velocity. Velocities on other components are ignored.

### Published Topic

* **`/omnidirectional_controller/odom`** ([nav_msgs/msg/Odometry])

    Robot odometry. The odometry can be computed from hardware feedback or using an open-loop approach, in which the integration is performed using the Twist command. You can select the approach in the configuration file. Additionally, you can choose either the Runge-Kutta or Euler Forward integration method.

## Unit Test

> TODO

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker]

[ros2_control]: https://control.ros.org/master/index.html
[ros2_controllers]: https://control.ros.org/master/doc/ros2_controllers/doc/controllers_index.html
[Issue Tracker]: https://github.com/mateusmenezes95/omnidirectional_controllers/issues
[diff_drive_controller]: https://control.ros.org/master/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html
[ros2_control URDF]: https://github.com/mateusmenezes95/axebot/blob/foxy/axebot_description/urdf/ros2_control.urdf.xacro
[controller configuration]: https://github.com/mateusmenezes95/axebot/blob/foxy/axebot_control/config/omnidirectional_controller.yaml
[launch file]: https://github.com/mateusmenezes95/axebot/blob/foxy/axebot_gazebo/launch/axebot.launch.py
[Creating a workspace tutorial]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html
[recommended ubuntu installation tutorial]: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
[Source the overlay]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#source-the-overlay
[geometry_msgs/msg/Twist]: https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html
[nav_msgs/msg/Odometry]: https://docs.ros2.org/latest/api/nav_msgs/msg/Odometry.html
[Classic Gazebo 11]: https://classic.gazebosim.org/
