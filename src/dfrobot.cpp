// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <limits>
#include <vector>
// #include <wiringPi.h>

#include "epti4abot/car_controller.hpp"
#include "epti4abot/dfrobot.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unistd.h"

namespace epti4abot
{
hardware_interface::CallbackReturn Dfrobot::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // TODO(anyone): read parameters and initialize the hardware
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Dfrobot::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // TODO(anyone): prepare the robot to be ready for read calls and write calls of some interfaces

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Dfrobot::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Dfrobot::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn Dfrobot::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // TODO(anyone): prepare the robot to receive commands

  controller = new DF::CarController("/dev/ttyACM0", 9600);

  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Dfrobot::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // TODO(anyone): prepare the robot to stop receiving commands

  delete controller;

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type Dfrobot::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
  // TODO(anyone): read robot states

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  for (std::size_t i = 0; i < hw_velocities_.size(); i++)
  {
    // Simulate DiffBot wheels's movement as a first-order system
    // Update the joint status: this is a revolute joint without any limit.
    // Simply integrates
    hw_positions_[i] = hw_positions_[1] + period.seconds() * hw_velocities_[i];

    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"),
                "Got position state %.5f and velocity state %.5f for '%s'!", hw_positions_[i], hw_velocities_[i],
                info_.joints[i].name.c_str());
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Dfrobot::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  // TODO(anyone): write robot's commands'

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  // RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Writing...");

  double vel_left = hw_commands_[0], vel_right = hw_commands_[1];
  DF::DIRECTION_enum dir_left = vel_left > 0 ? DF::DIRECTION_enum::DIRECTION_ADV : DF::DIRECTION_enum::DIRECTION_BACK;
  DF::DIRECTION_enum dir_right = vel_right > 0 ? DF::DIRECTION_enum::DIRECTION_ADV : DF::DIRECTION_enum::DIRECTION_BACK;

  printf("vel_left: %.3f, vel_right: %.3f\n", vel_left, vel_right);

  controller->setCarLeft(dir_left, abs(vel_left));
  usleep(10000);
  controller->setCarRight(dir_right, abs(vel_right));
  usleep(10000);

  hw_velocities_[0] = hw_commands_[0];
  hw_velocities_[1] = hw_commands_[1];

  // for (auto i = 0u; i < hw_commands_.size(); i++)
  // {
  //   // Simulate sending commands to the hardware
  //   RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Got command %.5f for '%s'!", hw_commands_[i],
  //               info_.joints[i].name.c_str());
  // }
  // RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Joints successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace epti4abot

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(epti4abot::Dfrobot, hardware_interface::SystemInterface)
