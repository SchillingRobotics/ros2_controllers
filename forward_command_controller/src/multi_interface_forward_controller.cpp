// Copyright (c) 2021, PickNik, Inc.
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

#include "forward_command_controller/multi_interface_forward_controller.hpp"

#include <string>
#include <vector>

namespace forward_command_controller
{

MultiInterfaceForwardController::MultiInterfaceForwardController()
: forward_command_controller::ForwardCommandController()
{
  logger_name_ = "multi interface forward controller";
}

CallbackReturn
MultiInterfaceForwardController::on_init()
{
  auto ret = ForwardCommandController::on_init();
  if (ret != CallbackReturn::SUCCESS)
  {
    return ret;
  }

  try {
    get_node()->set_parameter(rclcpp::Parameter("joints", std::vector<std::string>()));
    get_node()->set_parameter(rclcpp::Parameter("interface_name", ""));
    auto_declare<std::vector<std::string>>("joints_interfaces", joints_interfaces_);
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn MultiInterfaceForwardController::read_parameters()
{
  joints_interfaces_ = get_node()->get_parameter("joints_interfaces").as_string_array();

  if (joints_interfaces_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints_interfaces' parameter was empty");
    return CallbackReturn::ERROR;
  }

  command_interface_types_ = joints_interfaces_;

  return CallbackReturn::SUCCESS;
}

}  // namespace forward_command_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  forward_command_controller::MultiInterfaceForwardController,
  controller_interface::ControllerInterface)
