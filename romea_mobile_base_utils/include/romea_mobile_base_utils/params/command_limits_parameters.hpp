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

#ifndef ROMEA_MOBILE_BASE_UTILS__PARAMS__COMMAND_LIMITS_PARAMETERS_HPP_
#define ROMEA_MOBILE_BASE_UTILS__PARAMS__COMMAND_LIMITS_PARAMETERS_HPP_

// romea
#include <romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringCommandLimits.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringCommandLimits.hpp>
#include <romea_core_mobile_base/kinematic/omni_steering/OmniSteeringCommandLimits.hpp>
#include <romea_core_mobile_base/kinematic/skid_steering/SkidSteeringCommandLimits.hpp>

// romea ros
#include <romea_common_utils/params/ros_param.hpp>

namespace romea
{
namespace ros1
{

inline double get_minimal_longitudinal_speed(const ros::NodeHandle & nh)
{
  return load_param_or<double>(
    nh, "minimal_longitudinal_speed", -std::numeric_limits<double>::max());
}

inline double get_maximal_longitudinal_speed(const ros::NodeHandle & nh)
{
  return load_param_or<double>(
    nh, "maximal_longitudinal_speed", std::numeric_limits<double>::max());
}

inline double get_maximal_lateral_speed(const ros::NodeHandle & nh)
{
  return load_param_or<double>(nh, "maximal_lateral_speed", std::numeric_limits<double>::max());
}

inline double get_maximal_angular_speed(const ros::NodeHandle & nh)
{
  return load_param_or<double>(nh, "maximal_angular_speed", std::numeric_limits<double>::max());
}

inline double get_maximal_steering_angle(const ros::NodeHandle & nh)
{
  return load_param_or<double>(nh, "maximal_steering_angle", M_PI_2);
}

inline double get_maximal_front_steering_angle(const ros::NodeHandle & nh)
{
  return load_param_or<double>(nh, "maximal_front_steering_angle", M_PI_2);
}

inline double get_maximal_rear_steering_angle(const ros::NodeHandle & nh)
{
  return load_param_or<double>(nh, "maximal_rear_steering_angle", M_PI_2);
}

inline core::SkidSteeringCommandLimits get_skid_steering_command_limits(const ros::NodeHandle & nh)
{
  return core::SkidSteeringCommandLimits(
    get_minimal_longitudinal_speed(nh),
    get_maximal_longitudinal_speed(nh),
    get_maximal_angular_speed(nh));
}

inline core::OmniSteeringCommandLimits get_omni_steering_command_limits(const ros::NodeHandle & nh)
{
  return core::OmniSteeringCommandLimits(
    get_minimal_longitudinal_speed(nh),
    get_maximal_longitudinal_speed(nh),
    get_maximal_lateral_speed(nh),
    get_maximal_angular_speed(nh));
}

inline core::OneAxleSteeringCommandLimits get_one_axle_steering_command_limits(
  const ros::NodeHandle & nh)
{
  return core::OneAxleSteeringCommandLimits(
    get_minimal_longitudinal_speed(nh),
    get_maximal_longitudinal_speed(nh),
    get_maximal_steering_angle(nh));
}

inline core::TwoAxleSteeringCommandLimits get_two_axle_steering_command_limits(
  const ros::NodeHandle & nh)
{
  return core::TwoAxleSteeringCommandLimits(
    get_minimal_longitudinal_speed(nh),
    get_maximal_longitudinal_speed(nh),
    get_maximal_front_steering_angle(nh),
    get_maximal_rear_steering_angle(nh));
}

template<typename Limits, typename Node>
Limits get_command_limits(const ros::NodeHandle & nh)
{
  if constexpr (std::is_same_v<Limits, core::SkidSteeringCommandLimits>) {
    return get_skid_steering_command_limits(nh);
  } else if constexpr (std::is_same_v<Limits, core::OmniSteeringCommandLimits>) {
    return get_omni_steering_command_limits(nh);
  } else if constexpr (std::is_same_v<Limits, core::OneAxleSteeringCommandLimits>) {
    return get_one_axle_steering_command_limits(nh);
  } else if constexpr (std::is_same_v<Limits, core::TwoAxleSteeringCommandLimits>) {
    return get_two_axle_steering_command_limits(nh);
  }
}

}  // namespace ros1
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__PARAMS__COMMAND_LIMITS_PARAMETERS_HPP_
