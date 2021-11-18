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
//
/// \authors: Denis Stogl, Andy Zelenak

#include "admittance_controller/admittance_controller.hpp"

#include <math.h>
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <tf2_ros/buffer.h>

#include "admittance_controller/admittance_rule_impl.hpp"
#include "angles/angles.h"
#include "controller_interface/helpers.hpp"
#include "filters/filter_chain.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_limits/joint_limits_rosparam.hpp"
#include "rcutils/logging_macros.h"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

constexpr size_t ROS_LOG_THROTTLE_PERIOD = 1 * 1000;  // Milliseconds to throttle logs inside loops

namespace admittance_controller
{
AdmittanceController::AdmittanceController()
: controller_interface::ControllerInterface()
{
}

CallbackReturn AdmittanceController::on_init()
{
  try {
    // TODO: use variables as parameters
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<std::vector<std::string>>("command_interfaces", std::vector<std::string>());
    auto_declare<std::vector<std::string>>("state_interfaces", std::vector<std::string>());
    auto_declare<std::string>("ft_sensor_name", "");
    auto_declare<bool>("use_joint_commands_as_input", false);
    auto_declare<bool>("open_loop_control", false);
    auto_declare<std::string>("joint_limiter_type", "joint_limits/SimpleJointLimiter");
    auto_declare<double>("action_monitor_rate", 20.0);

    auto_declare<std::string>("IK.base", "");
    // TODO(destogl): enable when IK-plugin support is added
//     auto_declare<std::string>("IK.plugin", "");
    auto_declare<std::string>("IK.group_name", "");

    auto_declare<std::string>("control_frame", "");
    auto_declare<std::string>("sensor_frame", "");

    // TODO(destogl): enable when force/position control is implemented
//     auto_declare<bool>("admitance.unified_mode", false);
    auto_declare<bool>("admittance.selected_axes.x", false);
    auto_declare<bool>("admittance.selected_axes.y", false);
    auto_declare<bool>("admittance.selected_axes.z", false);
    auto_declare<bool>("admittance.selected_axes.rx", false);
    auto_declare<bool>("admittance.selected_axes.ry", false);
    auto_declare<bool>("admittance.selected_axes.rz", false);

    auto_declare<double>("admittance.mass.x", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>("admittance.mass.y", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>("admittance.mass.z", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>("admittance.mass.rx", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>("admittance.mass.ry", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>("admittance.mass.rz", std::numeric_limits<double>::quiet_NaN());

    auto_declare<double>("admittance.damping.x", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>("admittance.damping.y", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>("admittance.damping.z", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>("admittance.damping.rx", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>("admittance.damping.ry", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>("admittance.damping.rz", std::numeric_limits<double>::quiet_NaN());

    auto_declare<double>("admittance.damping_ratio.x", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>("admittance.damping_ratio.y", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>("admittance.damping_ratio.z", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>("admittance.damping_ratio.rx", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>("admittance.damping_ratio.ry", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>("admittance.damping_ratio.rz", std::numeric_limits<double>::quiet_NaN());

    auto_declare<double>("admittance.stiffness.x", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>("admittance.stiffness.y", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>("admittance.stiffness.z", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>("admittance.stiffness.rx", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>("admittance.stiffness.ry", std::numeric_limits<double>::quiet_NaN());
    auto_declare<double>("admittance.stiffness.rz", std::numeric_limits<double>::quiet_NaN());

    auto_declare<bool>("allow_partial_joints_goal", allow_partial_joints_goal_);
    auto_declare<bool>("open_loop_control", open_loop_control_);
    auto_declare<bool>(
      "allow_integration_in_goal_trajectories", allow_integration_in_goal_trajectories_);
    auto_declare<double>("constraints.stopped_velocity_tolerance", 0.01);
    auto_declare<double>("constraints.goal_time", 0.0);

  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  admittance_ = std::make_unique<admittance_controller::AdmittanceRule>();

  return CallbackReturn::SUCCESS;
}

CallbackReturn AdmittanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto error_if_empty = [&](const auto & parameter, const char * parameter_name) {
    if (parameter.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "'%s' parameter was empty", parameter_name);
      return true;
    }
    return false;
  };

  auto get_string_array_param_and_error_if_empty = [&](
    std::vector<std::string> & parameter, const char * parameter_name) {
    parameter = get_node()->get_parameter(parameter_name).as_string_array();
    return error_if_empty(parameter, parameter_name);
  };

  auto get_string_param_and_error_if_empty = [&](
    std::string & parameter, const char * parameter_name) {
    parameter = get_node()->get_parameter(parameter_name).as_string();
    return error_if_empty(parameter, parameter_name);
  };

  // TODO(destogl): If we would use C++20 than we can use templates here
  auto get_bool_param_and_error_if_empty = [&](
    bool & parameter, const char * parameter_name) {
    parameter = get_node()->get_parameter(parameter_name).get_value<bool>();
    return false; // TODO(destogl): how to check "if_empty" for bool?
  };

  auto get_double_param_and_error_if_empty = [&](
    double & parameter, const char * parameter_name) {
    parameter = get_node()->get_parameter(parameter_name).get_value<double>();
    if (std::isnan(parameter)) {
      RCLCPP_ERROR(get_node()->get_logger(), "'%s' parameter was not set", parameter_name);
      return true;
    }
    return false;
  };

  auto get_double_params_and_error_if_both_empty_or_set = [&](
    double & parameter, const char * parameter_name,
    double & parameter2, const char * parameter_name2) {
    parameter = get_node()->get_parameter(parameter_name).get_value<double>();
    parameter2 = get_node()->get_parameter(parameter_name2).get_value<double>();
    if (std::isnan(parameter) == std::isnan(parameter2)) {
      RCLCPP_ERROR(get_node()->get_logger(), "'%s' and '%s' parameters were both '%s'set",
                   parameter_name, parameter_name2, (std::isnan(parameter) ? "not " : ""));
      return true;
    }
    return false;
    };

  if (
    get_string_array_param_and_error_if_empty(joint_names_, "joints") ||
    get_string_array_param_and_error_if_empty(command_interface_types_, "command_interfaces") ||
    get_string_array_param_and_error_if_empty(state_interface_types_, "state_interfaces") ||
    get_string_param_and_error_if_empty(ft_sensor_name_, "ft_sensor_name") ||
    get_bool_param_and_error_if_empty(use_joint_commands_as_input_, "use_joint_commands_as_input") ||
    get_bool_param_and_error_if_empty(admittance_->open_loop_control_, "open_loop_control") ||
    get_string_param_and_error_if_empty(joint_limiter_type_, "joint_limiter_type") ||
    get_string_param_and_error_if_empty(admittance_->ik_base_frame_, "IK.base") ||
    get_string_param_and_error_if_empty(admittance_->ik_group_name_, "IK.group_name") ||
    get_string_param_and_error_if_empty(admittance_->control_frame_, "control_frame") ||
    get_string_param_and_error_if_empty(admittance_->sensor_frame_, "sensor_frame") ||

    get_bool_param_and_error_if_empty(admittance_->selected_axes_[0], "admittance.selected_axes.x") ||
    get_bool_param_and_error_if_empty(admittance_->selected_axes_[1], "admittance.selected_axes.y") ||
    get_bool_param_and_error_if_empty(admittance_->selected_axes_[2], "admittance.selected_axes.z") ||
    get_bool_param_and_error_if_empty(admittance_->selected_axes_[3], "admittance.selected_axes.rx") ||
    get_bool_param_and_error_if_empty(admittance_->selected_axes_[4], "admittance.selected_axes.ry") ||
    get_bool_param_and_error_if_empty(admittance_->selected_axes_[5], "admittance.selected_axes.rz") ||

    get_double_param_and_error_if_empty(admittance_->mass_[0], "admittance.mass.x") ||
    get_double_param_and_error_if_empty(admittance_->mass_[1], "admittance.mass.y") ||
    get_double_param_and_error_if_empty(admittance_->mass_[2], "admittance.mass.z") ||
    get_double_param_and_error_if_empty(admittance_->mass_[3], "admittance.mass.rx") ||
    get_double_param_and_error_if_empty(admittance_->mass_[4], "admittance.mass.ry") ||
    get_double_param_and_error_if_empty(admittance_->mass_[5], "admittance.mass.rz") ||

    get_double_params_and_error_if_both_empty_or_set(
      admittance_->damping_[0], "admittance.damping.x",
      admittance_->damping_ratio_[0], "admittance.damping_ratio.x") ||
    get_double_params_and_error_if_both_empty_or_set(
      admittance_->damping_[1], "admittance.damping.y",
      admittance_->damping_ratio_[1], "admittance.damping_ratio.y") ||
    get_double_params_and_error_if_both_empty_or_set(
      admittance_->damping_[2], "admittance.damping.z",
      admittance_->damping_ratio_[2], "admittance.damping_ratio.z") ||
    get_double_params_and_error_if_both_empty_or_set(
      admittance_->damping_[3], "admittance.damping.rx",
      admittance_->damping_ratio_[3], "admittance.damping_ratio.rx") ||
    get_double_params_and_error_if_both_empty_or_set(
      admittance_->damping_[4], "admittance.damping.ry",
      admittance_->damping_ratio_[4], "admittance.damping_ratio.ry") ||
    get_double_params_and_error_if_both_empty_or_set(
      admittance_->damping_[5], "admittance.damping.rz",
      admittance_->damping_ratio_[5], "admittance.damping_ratio.rz") ||

    get_double_param_and_error_if_empty(admittance_->stiffness_[0], "admittance.stiffness.x") ||
    get_double_param_and_error_if_empty(admittance_->stiffness_[1], "admittance.stiffness.y") ||
    get_double_param_and_error_if_empty(admittance_->stiffness_[2], "admittance.stiffness.z") ||
    get_double_param_and_error_if_empty(admittance_->stiffness_[3], "admittance.stiffness.rx") ||
    get_double_param_and_error_if_empty(admittance_->stiffness_[4], "admittance.stiffness.ry") ||
    get_double_param_and_error_if_empty(admittance_->stiffness_[5], "admittance.stiffness.rz")
    )
  {
    return CallbackReturn::ERROR;
  }

  // Allocate vector
  joint_deltas_.resize(joint_names_.size());

  // Convert the damping ratio (if given) to mass/spring/damper representation
  admittance_->convert_damping_ratio_to_damping();

  try {
    admittance_->filter_chain_ =
    std::make_unique<filters::FilterChain<geometry_msgs::msg::WrenchStamped>>(
      "geometry_msgs::msg::WrenchStamped");
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during filter chain creation at configure stage with message : %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }

  if (!admittance_->filter_chain_->configure("input_wrench_filter_chain",
    get_node()->get_node_logging_interface(), get_node()->get_node_parameters_interface()))
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Could not configure sensor filter chain, please check if the "
                 "parameters are provided correctly.");
    return CallbackReturn::ERROR;
  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  joint_command_interface_.resize(allowed_interface_types_.size());
  for (const auto & interface : command_interface_types_) {
    auto it = std::find(
      allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    if (it == allowed_interface_types_.end()) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Command interface type '" << interface << "' not allowed!");
      return CallbackReturn::ERROR;
    }
  }

  if (controller_interface::interface_list_contains_interface_type(
    command_interface_types_, hardware_interface::HW_IF_POSITION)) {
    has_position_command_interface_ = true;
  }
  if (controller_interface::interface_list_contains_interface_type(
    command_interface_types_, hardware_interface::HW_IF_VELOCITY)) {
    has_velocity_command_interface_ = true;
  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  joint_state_interface_.resize(allowed_interface_types_.size());
  for (const auto & interface : state_interface_types_) {
    auto it = std::find(
      allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    if (it == allowed_interface_types_.end()) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(), "State interface type '" << interface << "' not allowed!");
      return CallbackReturn::ERROR;
    }
  }

  if (!controller_interface::interface_list_contains_interface_type(
    state_interface_types_, hardware_interface::HW_IF_POSITION)) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "State interface type '" << std::string(hardware_interface::HW_IF_POSITION) << "' has to be always present allowed!");
    return CallbackReturn::ERROR;
  }

  if (controller_interface::interface_list_contains_interface_type(
    state_interface_types_, hardware_interface::HW_IF_VELOCITY)) {
    has_velocity_state_interface_ = true;
  }

  auto get_interface_list = [](const std::vector<std::string> & interface_types) {
    std::stringstream ss_command_interfaces;
    for (size_t index = 0; index < interface_types.size(); ++index) {
      if (index != 0) {
        ss_command_interfaces << " ";
      }
      ss_command_interfaces << interface_types[index];
    }
    return ss_command_interfaces.str();
  };

  // Print output so users can be sure the interface setup is correct
  RCLCPP_INFO(
    get_node()->get_logger(), "Command interfaces are [%s] and and state interfaces are [%s].",
    get_interface_list(command_interface_types_).c_str(),
              get_interface_list(state_interface_types_).c_str());

  auto num_joints = joint_names_.size();

  // Initialize joint limits
  if (!joint_limiter_type_.empty())
  {
    RCLCPP_INFO(
      get_node()->get_logger(), "Using joint limiter plugin: '%s'", joint_limiter_type_.c_str());
    joint_limiter_loader_ = std::make_shared<pluginlib::ClassLoader<JointLimiter>>(
      "joint_limits", "joint_limits::JointLimiterInterface<joint_limits::JointLimits>");
    joint_limiter_ = std::unique_ptr<JointLimiter>(
      joint_limiter_loader_->createUnmanagedInstance(joint_limiter_type_));
    joint_limiter_->init(joint_names_, get_node());
  }
  else
  {
    RCLCPP_INFO(
      get_node()->get_logger(), "Not using joint limiter plugin as none defined.");
  }

  default_tolerances_ = joint_trajectory_controller::get_segment_tolerances(*node_, joint_names_);

  // Read parameters customizing controller for special cases
  open_loop_control_ = node_->get_parameter("open_loop_control").get_value<bool>();
  allow_integration_in_goal_trajectories_ =
    node_->get_parameter("allow_integration_in_goal_trajectories").get_value<bool>();

  // subscriber callback
  // non realtime
  // TODO(karsten): check if traj msg and point time are valid
  auto callback = [this](const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg) -> void {
    if (!validate_trajectory_msg(
          *msg, allow_partial_joints_goal_, joint_names_, allow_integration_in_goal_trajectories_,
          node_->now()))
    {
      return;
    }

    // http://wiki.ros.org/joint_trajectory_controller/UnderstandingTrajectoryReplacement
    // always replace old msg with new one for now
    if (subscriber_is_active_)
    {
      add_new_trajectory_msg(msg);
    }
  };

  // TODO(karsten1987): create subscriber with subscription deactivated
  joint_command_subscriber_ = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "~/joint_trajectory", rclcpp::SystemDefaultsQoS(), callback);

  // Initialize FTS semantic semantic_component
  force_torque_sensor_ = std::make_unique<semantic_components::ForceTorqueSensor>(
    semantic_components::ForceTorqueSensor(ft_sensor_name_));

  // TODO(destogl): Add subscriber for velocity scaling

  // State publisher
  s_publisher_ = get_node()->create_publisher<ControllerStateMsg>(
    "~/state", rclcpp::SystemDefaultsQoS());
  state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);

  // Initialize state message
  state_publisher_->lock();
  state_publisher_->msg_.joint_names = joint_names_;
  state_publisher_->msg_.actual_joint_state.positions.resize(num_joints, 0.0);
  state_publisher_->msg_.desired_joint_state.positions.resize(num_joints, 0.0);
  state_publisher_->msg_.error_joint_state.positions.resize(num_joints, 0.0);
  state_publisher_->unlock();

  // Configure AdmittanceRule
  admittance_->configure(get_node());

  last_commanded_state_.positions.resize(num_joints);
  last_commanded_state_.velocities.resize(num_joints, 0.0);
  last_commanded_state_.accelerations.resize(num_joints, 0.0);

  if (use_joint_commands_as_input_) {
    RCLCPP_INFO(get_node()->get_logger(), "Using Joint input mode.");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Admittance controller does not support non-joint input modes.");
    return CallbackReturn::ERROR;
  }

  // action server configuration
  allow_partial_joints_goal_ = node_->get_parameter("allow_partial_joints_goal").get_value<bool>();
  if (allow_partial_joints_goal_)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Goals with partial set of joints are allowed");
  }

  const double action_monitor_rate =
    node_->get_parameter("action_monitor_rate").get_value<double>();

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration AdmittanceController::command_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(joint_names_.size() * command_interface_types_.size());
  for (const auto & joint : joint_names_) {
    for (const auto & interface : command_interface_types_) {
      command_interfaces_config.names.push_back(joint + "/" + interface);
    }
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration AdmittanceController::state_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(6 + joint_names_.size() * state_interface_types_.size());

  state_interfaces_config.names = force_torque_sensor_->get_state_interface_names();

  for (const auto & joint : joint_names_) {
    for (const auto & interface : state_interface_types_) {
      state_interfaces_config.names.push_back(joint + "/" + interface);
    }
  }

  return state_interfaces_config;
}

CallbackReturn AdmittanceController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  const auto num_joints = joint_names_.size();

  // order all joints in the storage
  for (const auto & interface : command_interface_types_) {
    auto it = std::find(
      allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = std::distance(allowed_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(
      command_interfaces_, joint_names_, interface, joint_command_interface_[index]))
    {
      RCLCPP_ERROR(
        node_->get_logger(), "Expected %zu '%s' command interfaces, got %zu.",
                   num_joints, interface.c_str(), joint_command_interface_[index].size());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
  }
  for (const auto & interface : state_interface_types_) {
    auto it = std::find(
      allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = std::distance(allowed_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(
      state_interfaces_, joint_names_, interface, joint_state_interface_[index]))
    {
      RCLCPP_ERROR(
        node_->get_logger(), "Expected %zu '%s' state interfaces, got %zu.",
                   num_joints, interface.c_str(), joint_state_interface_[index].size());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
  }

  // Store 'home' pose
  traj_msg_home_ptr_ = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  traj_msg_home_ptr_->header.stamp.sec = 0;
  traj_msg_home_ptr_->header.stamp.nanosec = 0;
  traj_msg_home_ptr_->points.resize(1);
  traj_msg_home_ptr_->points[0].time_from_start.sec = 0;
  traj_msg_home_ptr_->points[0].time_from_start.nanosec = 50000000;
  traj_msg_home_ptr_->points[0].positions.resize(joint_state_interface_[0].size());
  for (size_t index = 0; index < joint_state_interface_[0].size(); ++index)
  {
    traj_msg_home_ptr_->points[0].positions[index] =
      joint_state_interface_[0][index].get().get_value();
  }

  traj_external_point_ptr_ = std::make_shared<joint_trajectory_controller::Trajectory>();
  traj_home_point_ptr_ = std::make_shared<joint_trajectory_controller::Trajectory>();
  traj_msg_external_point_ptr_.writeFromNonRT(
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory>());

  subscriber_is_active_ = true;
  traj_point_active_ptr_ = &traj_external_point_ptr_;

  // Initialize current state storage if hardware state has tracking offset
  resize_joint_trajectory_point(
    last_commanded_state_, joint_names_.size(), has_velocity_state_interface_,
    has_acceleration_state_interface_);
  read_state_from_hardware(last_commanded_state_);
  // Handle restart of controller by reading last_commanded_state_ from commands is
  // those are not nan
  trajectory_msgs::msg::JointTrajectoryPoint state;
  resize_joint_trajectory_point(
    state, joint_names_.size(), has_velocity_state_interface_, has_acceleration_state_interface_);

  if (read_state_from_command_interfaces(state))
  {
    last_commanded_state_ = state;
  }

  // Initialize interface of the FTS semantic semantic component
  force_torque_sensor_->assign_loaned_state_interfaces(state_interfaces_);

  // Initialize Admittance Rule from current states
  admittance_->reset();
  previous_time_ = get_node()->now();

  read_state_from_hardware(last_commanded_state_);
  if (joint_limiter_)
  {
    joint_limiter_->configure(last_commanded_state_);
  }
  // Handle restart of controller by reading last_commanded_state_ from commands if not nan
  read_state_from_command_interfaces(last_commanded_state_);

  prev_trajectory_point_ = last_commanded_state_;

  // Set initial command values - initialize all to simplify update
  std::shared_ptr<ControllerCommandWrenchMsg> msg_wrench = std::make_shared<ControllerCommandWrenchMsg>();
  msg_wrench->header.frame_id = admittance_->control_frame_;
  input_wrench_command_.writeFromNonRT(msg_wrench);

  std::shared_ptr<ControllerCommandJointMsg> msg_joint = std::make_shared<ControllerCommandJointMsg>();
  msg_joint->joint_names = joint_names_;
  msg_joint->points.reserve(1);

  trajectory_msgs::msg::JointTrajectoryPoint trajectory_point;
  trajectory_point.positions.reserve(num_joints);
  trajectory_point.velocities.resize(num_joints, 0.0);
  // FIXME(destogl): ATTENTION: This does not work properly, so using velocity mode and commenting positions out!
//   for (auto index = 0u; index < num_joints; ++index) {
//     trajectory_point.positions.emplace_back(joint_state_interface_[0][index].get().get_value());
//   }
  msg_joint->points.emplace_back(trajectory_point);

  input_joint_command_.writeFromNonRT(msg_joint);

  std::shared_ptr<ControllerCommandPoseMsg> msg_pose = std::make_shared<ControllerCommandPoseMsg>();
  msg_pose->header.frame_id = admittance_->control_frame_;
  if (admittance_->get_pose_of_control_frame_in_base_frame(*msg_pose) !=
      controller_interface::return_type::OK)
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "Can not find transform from '%s' to '%s' needed in the update loop",
                 admittance_->ik_base_frame_.c_str(), admittance_->control_frame_.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  input_pose_command_.writeFromNonRT(msg_pose);

  const double action_monitor_rate =
    node_->get_parameter("action_monitor_rate").get_value<double>();

  create_action_server(
    node_, this, action_monitor_rate, allow_partial_joints_goal_, joint_names_,
    allow_integration_in_goal_trajectories_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn AdmittanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop the robot
  for (auto index = 0ul; index < joint_names_.size(); ++index) {
    joint_command_interface_[0][index].get().set_value(
      joint_command_interface_[0][index].get().get_value());
  }

  for (auto index = 0ul; index < allowed_interface_types_.size(); ++index) {
    joint_command_interface_[index].clear();
    joint_state_interface_[index].clear();
  }
  release_interfaces();

  release_action_server();

  force_torque_sensor_->release_interfaces();

  subscriber_is_active_ = false;

  return CallbackReturn::SUCCESS;
}

CallbackReturn AdmittanceController::on_cleanup(const rclcpp_lifecycle::State &)
{
  // go home
  traj_home_point_ptr_->update(traj_msg_home_ptr_);
  traj_point_active_ptr_ = &traj_home_point_ptr_;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

CallbackReturn AdmittanceController::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type AdmittanceController::update()
{
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    return controller_interface::return_type::OK;
  }

  // Check if a new external message has been received from nonRT threads
  check_for_new_trajectory(joint_names_, joint_command_interface_);

  trajectory_msgs::msg::JointTrajectoryPoint state_current, state_desired, state_error;
  trajectory_msgs::msg::JointTrajectory pre_admittance_point;
  const auto joint_num = joint_names_.size();
  resize_joint_trajectory_point(
    state_current, joint_num, has_velocity_state_interface_, has_acceleration_state_interface_);
  resize_joint_trajectory_point(
    state_error, joint_num, has_velocity_state_interface_, has_acceleration_state_interface_);

  // current state update
  state_current.time_from_start.set__sec(0);
  read_state_from_hardware(state_current);

  if (admittance_->open_loop_control_) {
    // TODO(destogl): This may not work in every case.
    // Please add checking which states are available and which not!
    state_current = last_commanded_state_;
  }

  // admittance rule
  geometry_msgs::msg::Wrench ft_values;
  force_torque_sensor_->get_values_as_message(ft_values);

  const size_t num_joints = joint_names_.size();

  // TODO(anyone): can I here also use const on joint_interface since the reference_wrapper is not
  // changed, but its value only?
  auto assign_interface_from_point =
    [&, joint_num](auto & joint_inteface, const std::vector<double> & trajectory_point_interface) {
      for (auto index = 0ul; index < joint_num; ++index)
      {
        joint_inteface[index].get().set_value(trajectory_point_interface[index]);
      }
    };

  bool valid_trajectory_point = false;
  // Used only when we have a valid trajectory
  bool before_last_point = false;
  bool abort = false;
  bool outside_goal_state_tolerance = false;
  joint_trajectory_controller::TrajectoryPointConstIter start_segment_itr, end_segment_itr;

  // currently carrying out a trajectory
  if (have_trajectory())
  {
    // if we will be sampling for the first time, prefix the trajectory with the current state
    set_point_before_trajectory_msg(
      open_loop_control_, node_->now(), state_current, prev_trajectory_point_);

    // find segment for current timestamp
    // joint_trajectory_controller::TrajectoryPointConstIter start_segment_itr, end_segment_itr;

    valid_trajectory_point = sample_trajectory(
      node_->now(), state_desired, start_segment_itr, end_segment_itr);

    before_last_point = is_before_last_point(end_segment_itr);

    if (valid_trajectory_point) prev_trajectory_point_ = state_desired;
  }

  auto duration_since_last_call = get_node()->now() - previous_time_;
  // TODO: Use pre-allocated joint_deltas_ vector 
  std::array<double, 6> joint_deltas;

  // Here we have no servo or trajectory command, maintain current position
  if (!valid_trajectory_point) {
    state_desired = prev_trajectory_point_;
    state_desired.velocities.assign(num_joints, 0.0);
    state_desired.accelerations.assign(num_joints, 0.0);
  }

  // If there are no positions, expect velocities
  // TODO(destogl): add error handling
  // if (state_desired.positions.empty()) {
  //   for (auto index = 0u; index < num_joints; ++index) {
  //     joint_deltas.at(index) = state_desired.velocities[index] * duration_since_last_call.seconds();
  //   }
  // } else {
  //   for (auto index = 0u; index < num_joints; ++index) {
  //     // TODO(destogl): ATTENTION: This does not work properly, deltas are getting neutralized and robot is not moving on external forces
  //     // TODO(anyone): Is here OK to use shortest_angular_distance?
  //     joint_deltas.at(index) = angles::shortest_angular_distance(state_current.positions[index], state_desired.positions[index]);
  //   }
  // }
  // Always use velocities since positions aren't working
  for (auto index = 0u; index < num_joints; ++index) {
    joint_deltas.at(index) = state_desired.velocities[index] * duration_since_last_call.seconds();
  }

  pre_admittance_point.points.push_back(state_desired);

  admittance_->update(state_current, ft_values, joint_deltas, duration_since_last_call, state_desired);

  // Apply joint_limiter_
  if (joint_limiter_)
  {
    joint_limiter_->enforce(state_current, state_desired, duration_since_last_call);
  }

   // Compute state_error
  auto compute_error_for_joint = [&](
                                   trajectory_msgs::msg::JointTrajectoryPoint & error, int index,
                                   const trajectory_msgs::msg::JointTrajectoryPoint & current,
                                   const trajectory_msgs::msg::JointTrajectoryPoint & desired) {
    // error defined as the difference between current and desired
    error.positions[index] =
      angles::shortest_angular_distance(current.positions[index], desired.positions[index]);
    if (has_velocity_state_interface_ && has_velocity_command_interface_)
    {
      error.velocities[index] = desired.velocities[index] - current.velocities[index];
    }
    if (has_acceleration_state_interface_ && has_acceleration_command_interface_)
    {
      error.accelerations[index] = desired.accelerations[index] - current.accelerations[index];
    }
  };

  for (auto index = 0ul; index < joint_num; ++index)
  {
    compute_error_for_joint(state_error, index, state_current, state_desired);

    // Check if trajectory is complete or should be aborted
    if(valid_trajectory_point) {
      if (
        before_last_point &&
        !check_state_tolerance_per_joint(
          state_error, index, default_tolerances_.state_tolerance[index], false))
      {
        abort = true;
      }
      // past the final point, check that we end up inside goal tolerance
      if (
        !before_last_point &&
        !check_state_tolerance_per_joint(
          state_error, index, default_tolerances_.goal_state_tolerance[index], false))
      {
        outside_goal_state_tolerance = true;
      }
    }
  }

  // Write new joint angles to the robot
  // set values for next hardware write()
  if (has_position_command_interface_)
  {
    assign_interface_from_point(joint_command_interface_[0], state_desired.positions);
  }
  if (has_velocity_command_interface_)
  {
    assign_interface_from_point(joint_command_interface_[1], state_desired.velocities);
  }
  if (has_acceleration_command_interface_)
  {
    assign_interface_from_point(joint_command_interface_[2], state_desired.accelerations);
  }
  // TODO(anyone): Add here "if using_closed_loop_hw_interface_adapter" (see ROS1) - #171
  //       if (check_if_interface_type_exist(
  //           command_interface_types_, hardware_interface::HW_IF_EFFORT)) {
  //         assign_interface_from_point(joint_command_interface_[3], state_desired.effort);
  //       }

  if(valid_trajectory_point) {
    // handle action server feedback
    perform_action_server_update(
      before_last_point, abort, outside_goal_state_tolerance,
      default_tolerances_.goal_time_tolerance, node_->now(), joint_names_, state_current,
      state_desired, state_error, start_segment_itr);
  }

  // store command as state when hardware state has tracking offset
  // TODO: Should we be checking if open_loop_control_ around this assignment?
  last_commanded_state_ = state_desired;
  previous_time_ = get_node()->now();

  // Publish controller state
  state_publisher_->lock();
  state_publisher_->msg_.input_joint_command = pre_admittance_point;
  state_publisher_->msg_.desired_joint_state = state_desired;
  state_publisher_->msg_.actual_joint_state = state_current;
  state_publisher_->msg_.error_joint_state = state_error;
  admittance_->get_controller_state(state_publisher_->msg_);
  state_publisher_->unlockAndPublish();

  return controller_interface::return_type::OK;
}

void AdmittanceController::read_state_from_hardware(
  trajectory_msgs::msg::JointTrajectoryPoint & state)
{
  const auto num_joints = joint_names_.size();
  auto assign_point_from_interface = [&, num_joints](
    std::vector<double> & trajectory_point_interface, const auto & joint_interface)
  {
    for (auto index = 0ul; index < num_joints; ++index) {
      trajectory_point_interface[index] = joint_interface[index].get().get_value();
    }
  };

  // Assign values from the hardware
  // Position states always exist
  assign_point_from_interface(state.positions, joint_state_interface_[0]);
  // velocity and acceleration states are optional
  if (has_velocity_state_interface_) {
    assign_point_from_interface(state.velocities, joint_state_interface_[1]);
    // Acceleration is used only in combination with velocity
    // TODO(destogl): enable acceleration and remove next line
    state.accelerations.clear();
//     if (has_acceleration_state_interface_) {
//       assign_point_from_interface(state.accelerations, joint_state_interface_[2]);
//     } else {
//       // Make empty so the property is ignored during interpolation
//       state.accelerations.clear();
//     }
  } else {
    // Make empty so the property is ignored during interpolation
    state.velocities.clear();
    state.accelerations.clear();
  }
}

bool AdmittanceController::read_state_from_command_interfaces(
  trajectory_msgs::msg::JointTrajectoryPoint & output_state)
{
  bool has_values = true;
  const auto num_joints = joint_names_.size();
  trajectory_msgs::msg::JointTrajectoryPoint state = output_state;

  auto assign_point_from_interface = [&, num_joints](
    std::vector<double> & trajectory_point_interface, const auto & joint_interface)
    {
      for (auto index = 0ul; index < num_joints; ++index) {
        trajectory_point_interface[index] = joint_interface[index].get().get_value();
      }
    };

  auto interface_has_values = [](const auto & joint_interface)
    {
      return std::find_if(
        joint_interface.begin(), joint_interface.end(),
        [](const auto & interface) {return std::isnan(interface.get().get_value());}) ==
             joint_interface.end();
    };

  // Assign values from the command interfaces as state. Therefore needs check for both.
  // Position state interface has to exist always
  if (has_position_command_interface_ && interface_has_values(joint_command_interface_[0])) {
    assign_point_from_interface(state.positions, joint_command_interface_[0]);
  } else {
    state.positions.clear();
    has_values = false;
  }
  // velocity and acceleration states are optional
  if (has_velocity_state_interface_) {
    if (has_velocity_command_interface_ && interface_has_values(joint_command_interface_[1])) {
      assign_point_from_interface(state.velocities, joint_command_interface_[1]);
      //TODO(destogl): enable this line under to be sure if positions are not existing and velocities
      // are existing to still update the output_state; !commented because not tested!
//       has_values = true;
    } else {
      state.velocities.clear();
      has_values = false;
    }
  }
  else {
    state.velocities.clear();
  }

// TODO(destogl): Enable this
//   // Acceleration is used only in combination with velocity
//   if (has_acceleration_state_interface_) {
//     if (has_acceleration_command_interface_ && interface_has_values(joint_command_interface_[2])) {
//       assign_point_from_interface(state.accelerations, joint_command_interface_[2]);
//     } else {
//       state.accelerations.clear();
//       has_values = false;
//     }
//   } else {
//     state.accelerations.clear();
//   }

  if (has_values) {
    output_state = state;
  }

  return has_values;
}

}  // namespace admittance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  admittance_controller::AdmittanceController,
  controller_interface::ControllerInterface)
