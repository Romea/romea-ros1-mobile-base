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

#ifndef ROMEA_MOBILE_BASE_UTILS__CONVERSIONS__KINEMATIC_CONVERSIONS_HPP_
#define ROMEA_MOBILE_BASE_UTILS__CONVERSIONS__KINEMATIC_CONVERSIONS_HPP_

// std
#include <string>

// romea
#include <romea_mobile_base_msgs/KinematicMeasureStamped.h>
#include <romea_mobile_base_msgs/OmniSteeringCommand.h>
#include <romea_mobile_base_msgs/OmniSteeringMeasureStamped.h>
#include <romea_mobile_base_msgs/OneAxleSteeringCommand.h>
#include <romea_mobile_base_msgs/OneAxleSteeringMeasureStamped.h>
#include <romea_mobile_base_msgs/SkidSteeringCommand.h>
#include <romea_mobile_base_msgs/SkidSteeringMeasureStamped.h>
#include <romea_mobile_base_msgs/TwoAxleSteeringCommand.h>
#include <romea_mobile_base_msgs/TwoAxleSteeringMeasureStamped.h>

#include <romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringCommand.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringMeasure.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringCommand.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringMeasure.hpp>
#include <romea_core_mobile_base/kinematic/omni_steering/OmniSteeringCommand.hpp>
#include <romea_core_mobile_base/kinematic/omni_steering/OmniSteeringMeasure.hpp>
#include <romea_core_mobile_base/kinematic/skid_steering/SkidSteeringCommand.hpp>
#include <romea_core_mobile_base/kinematic/skid_steering/SkidSteeringMeasure.hpp>

// ros
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <four_wheel_steering_msgs/FourWheelSteeringStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <ros/ros.h>

namespace romea
{
namespace ros1
{

core::KinematicMeasure to_romea(const romea_mobile_base_msgs::KinematicMeasure & msg);

void to_ros_msg(
  const core::KinematicMeasure & romea_kinematic_measure,
  romea_mobile_base_msgs::KinematicMeasure & ros_kinematic_msg);

void to_ros_msg(
  const ros::Time & stamp,
  const std::string & frame_id,
  const core::KinematicMeasure & romea_kinematic_measure,
  romea_mobile_base_msgs::KinematicMeasureStamped & ros_kinematic_stamped_msg);

void to_ros_msg(
  const core::KinematicMeasure & romea_kinematic_measure,
  geometry_msgs::TwistWithCovariance & ros_twist_with_covariance);

core::OneAxleSteeringMeasure to_romea(const romea_mobile_base_msgs::OneAxleSteeringMeasure & msg);

void to_ros_msg(
  const core::OneAxleSteeringMeasure & romea_one_axle_steering_measure,
  romea_mobile_base_msgs::OneAxleSteeringMeasure & ros_one_axle_steering_measure_msg);

void to_ros_msg(
  const ros::Time & stamp,
  const std::string & frame_id,
  const core::OneAxleSteeringMeasure & romea_one_axle_steering_measure,
  romea_mobile_base_msgs::OneAxleSteeringMeasureStamped & ros_one_axle_steering_measure_msg);

core::TwoAxleSteeringMeasure to_romea(const romea_mobile_base_msgs::TwoAxleSteeringMeasure & msg);

void to_ros_msg(
  const core::TwoAxleSteeringMeasure & romea_two_axle_steering_measure,
  romea_mobile_base_msgs::TwoAxleSteeringMeasure & ros_two_axle_steering_measure_msg);

void to_ros_msg(
  const ros::Time & stamp,
  const std::string & frame_id,
  const core::TwoAxleSteeringMeasure & romea_two_axle_steering_measure,
  romea_mobile_base_msgs::TwoAxleSteeringMeasureStamped & ros_two_axle_steering_measure_msg);

core::SkidSteeringMeasure to_romea(const romea_mobile_base_msgs::SkidSteeringMeasure & msg);

void to_ros_msg(
  const core::SkidSteeringMeasure & romea_skid_steering_measure,
  romea_mobile_base_msgs::SkidSteeringMeasure & ros_skid_steering_measure_msg);

void to_ros_msg(
  const ros::Time & stamp,
  const std::string & frame_id,
  const core::SkidSteeringMeasure & romea_skid_steering_measure,
  romea_mobile_base_msgs::SkidSteeringMeasureStamped & ros_skid_steering_measure_msg);

core::OmniSteeringMeasure to_romea(const romea_mobile_base_msgs::OmniSteeringMeasure & msg);

void to_ros_msg(
  const core::OmniSteeringMeasure & romea_omni_steering_measure,
  romea_mobile_base_msgs::OmniSteeringMeasure & ros_omni_steering_measure_msg);

void to_ros_msg(
  const ros::Time & stamp,
  const std::string & frame_id,
  const core::OmniSteeringMeasure & romea_omni_steering_measure,
  romea_mobile_base_msgs::OmniSteeringMeasureStamped & ros_omni_steering_measure_msg);

}  // namespace ros1
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__CONVERSIONS__KINEMATIC_CONVERSIONS_HPP_
