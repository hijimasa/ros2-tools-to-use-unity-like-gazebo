// Copyright 2020 PAL Robotics S.L.
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

/*
 * Author: Bence Magyar, Enrique Fernández, Manuel Meraz
 */

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "differential_system_controller/differential_system_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}  // namespace

namespace differential_system_controller
{
using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::HW_IF_EFFORT;
using lifecycle_msgs::msg::State;

DifferentialSystemController::DifferentialSystemController() : controller_interface::ControllerInterface() {}

const char * DifferentialSystemController::feedback_type() const
{
  return HW_IF_EFFORT;
}

controller_interface::CallbackReturn DifferentialSystemController::on_init()
{
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration DifferentialSystemController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  conf_names.push_back(params_.left_joint_name + "/" + HW_IF_VELOCITY);
  conf_names.push_back(params_.right_joint_name + "/" + HW_IF_VELOCITY);
  
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration DifferentialSystemController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  conf_names.push_back(params_.left_joint_name + "/" + feedback_type());
  conf_names.push_back(params_.right_joint_name + "/" + feedback_type());

  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type DifferentialSystemController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{

  auto logger = get_node()->get_logger();
  if (get_state().id() == State::PRIMARY_STATE_INACTIVE)
  {
    if (!is_halted)
    {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }
  
  float force_sum = 0.0f;
  for (size_t index = 0; index < 2; ++index)
  {
    force_sum += registered_joint_handles_[index].feedback.get().get_value();
  }
  current_joint_velocity_ = -force_sum * params_.force_constant / 100.0;
  for (size_t index = 0; index < 2; ++index)
  {
    registered_joint_handles_[index].velocity.get().set_value(current_joint_velocity_);
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn DifferentialSystemController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  // update parameters if they have changed
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(logger, "Parameters were updated");
  }

  if (params_.left_joint_name == "")
  {
    RCLCPP_ERROR(logger, "Left joint name parameters are empty!");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.right_joint_name == "")
  {
    RCLCPP_ERROR(logger, "Right joint name parameters are empty!");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DifferentialSystemController::on_activate(
  const rclcpp_lifecycle::State &)
{
  std::vector<std::string> joint_names;
  joint_names.push_back(params_.left_joint_name);
  joint_names.push_back(params_.right_joint_name);

  const auto wheel_config_result =
    configure_handles(joint_names, registered_joint_handles_);
  current_joint_velocity_ = 0.0;

  if (
    wheel_config_result == controller_interface::CallbackReturn::ERROR
  )
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (registered_joint_handles_.empty())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Joint interfaces are non existent");
    return controller_interface::CallbackReturn::ERROR;
  }

  is_halted = false;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DifferentialSystemController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (!is_halted)
  {
    halt();
    is_halted = true;
  }
  registered_joint_handles_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DifferentialSystemController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DifferentialSystemController::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool DifferentialSystemController::reset()
{
  registered_joint_handles_.clear();

  is_halted = false;
  return true;
}

controller_interface::CallbackReturn DifferentialSystemController::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

void DifferentialSystemController::halt()
{
  for (const auto & wheel_handle : registered_joint_handles_)
  {
    wheel_handle.velocity.get().set_value(0.0);
  }
}

controller_interface::CallbackReturn DifferentialSystemController::configure_handles(
  const std::vector<std::string> & wheel_names,
  std::vector<JointHandle> & registered_handles)
{
  auto logger = get_node()->get_logger();

  if (wheel_names.empty())
  {
    RCLCPP_ERROR(logger, "No omni wheel names specified");
    return controller_interface::CallbackReturn::ERROR;
  }

  // register handles
  registered_handles.reserve(wheel_names.size());
  for (const auto & wheel_name : wheel_names)
  {
    const auto interface_name = feedback_type();
    const auto state_handle = std::find_if(
      state_interfaces_.cbegin(), state_interfaces_.cend(),
      [&wheel_name, &interface_name](const auto & interface)
      {
        return interface.get_prefix_name() == wheel_name &&
               interface.get_interface_name() == interface_name;
      });

    if (state_handle == state_interfaces_.cend())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", wheel_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    const auto command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&wheel_name](const auto & interface)
      {
        return interface.get_prefix_name() == wheel_name &&
               interface.get_interface_name() == HW_IF_VELOCITY;
      });

    if (command_handle == command_interfaces_.end())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    registered_handles.emplace_back(
      JointHandle{std::ref(*state_handle), std::ref(*command_handle)});
  }

  return controller_interface::CallbackReturn::SUCCESS;
}
}  // namespace differential_system_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  differential_system_controller::DifferentialSystemController, controller_interface::ControllerInterface)
