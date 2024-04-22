// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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


#ifndef ROMEA_MOBILE_BASE_UTILS__CONVERSIONS__COMMAND_CONVERSIONS_HPP_
#define ROMEA_MOBILE_BASE_UTILS__CONVERSIONS__COMMAND_CONVERSIONS_HPP_

// romea
#include <romea_core_mobile_base/kinematic/skid_steering/SkidSteeringCommand.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringCommand.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringCommand.hpp>
#include <romea_core_mobile_base/kinematic/omni_steering/OmniSteeringCommand.hpp>
#include <romea_mobile_base_msgs/OneAxleSteeringCommand.h>
#include <romea_mobile_base_msgs/TwoAxleSteeringCommand.h>
#include <romea_mobile_base_msgs/SkidSteeringCommand.h>
#include <romea_mobile_base_msgs/OmniSteeringCommand.h>

// ros
#include <geometry_msgs/Twist.h>
#include <four_wheel_steering_msgs/FourWheelSteering.h>
#include <ackermann_msgs/AckermannDrive.h>

namespace romea
{
namespace ros1
{


void to_ros_msg(
  const core::TwoAxleSteeringCommand & romea_two_axle_steering_command,
  four_wheel_steering_msgs::FourWheelSteering & ros_four_wheel_steering_msg);

void to_romea(
  const four_wheel_steering_msgs::FourWheelSteering & ros_four_wheel_steering_msg,
  core::TwoAxleSteeringCommand & romea_two_axle_steering_command);

void to_ros_msg(
  const core::OneAxleSteeringCommand & romea_one_axle_steering_command,
  ackermann_msgs::AckermannDrive & ros_ackerman_drive_msg);

void to_romea(
  const ackermann_msgs::AckermannDrive & ros_ackerman_drive_msg,
  core::OneAxleSteeringCommand & romea_one_axle_steering_command);

void to_ros_msg(
  const core::OneAxleSteeringCommand & romea_one_axle_steering_command,
  geometry_msgs::Twist & ros_twist_msg);

void to_romea(
  const geometry_msgs::Twist & ros_twist_msg,
  core::OneAxleSteeringCommand & romea_one_axle_steering_command);

void to_ros_msg(
  const core::SkidSteeringCommand & romea_skid_steering_command,
  geometry_msgs::Twist & ros_twist_msg);

void to_romea(
  const geometry_msgs::Twist & ros_twist_msg,
  core::SkidSteeringCommand & romea_skid_steering_command);

void to_ros_msg(
  const core::OmniSteeringCommand & romea_omni_steering_command,
  geometry_msgs::Twist & ros_twist_msg);

void to_romea(
  const geometry_msgs::Twist & ros_twist_msg,
  core::OmniSteeringCommand & romea_omni_steering_command);

void to_ros_msg(
  const core::TwoAxleSteeringCommand & romea_two_axle_steering_command,
  romea_mobile_base_msgs::TwoAxleSteeringCommand & ros_two_axle_steering_command_msg);

void to_romea(
  const romea_mobile_base_msgs::TwoAxleSteeringCommand & ros_two_axle_steering_command_msg,
  core::TwoAxleSteeringCommand & romea_two_axle_steering_command);

void to_ros_msg(
  const core::OneAxleSteeringCommand & romea_one_axle_steering_command,
  romea_mobile_base_msgs::OneAxleSteeringCommand & ros_oxe_axle_steering_command_msg);

void to_romea(
  const romea_mobile_base_msgs::OneAxleSteeringCommand & ros_one_axle_steering_command_msg,
  core::OneAxleSteeringCommand & romea_one_axle_steering_command);

void to_ros_msg(
  const core::SkidSteeringCommand & romea_skid_steering_command,
  romea_mobile_base_msgs::SkidSteeringCommand & romea_skid_steering_command_msg);

void to_romea(
  const romea_mobile_base_msgs::SkidSteeringCommand & ros_skid_steering_command_msg,
  core::SkidSteeringCommand & romea_skid_steering_command);

void to_ros_msg(
  const core::OmniSteeringCommand & romea_omni_steering_command,
  romea_mobile_base_msgs::OmniSteeringCommand & romea_omni_steering_command_msg);

void to_romea(
  const romea_mobile_base_msgs::OmniSteeringCommand & ros_omni_steering_command_msg,
  core::OmniSteeringCommand & romea_omni_steering_command);


}  // namespace ros1
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__CONVERSIONS__COMMAND_CONVERSIONS_HPP_
