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

#ifndef ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_CONTROL_PARAMETERS_HPP_
#define ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_CONTROL_PARAMETERS_HPP_

// ros
#include <ros/ros.h>

// romea
#include <romea_common_utils/params/ros_param.hpp>
#include <romea_core_mobile_base/info/MobileBaseControl.hpp>

namespace romea
{
namespace ros1
{

inline core::SteeringAngleControl get_steering_angle_control_info(const ros::NodeHandle& nh)
{
  return {
    {load_param<double>(nh, "sensor/angle_std"),
     load_param<double>(nh, "sensor/angle_range")},
    {load_param<double>(nh, "command/maximal_angle"),
     load_param<double>(nh, "command/maximal_angular_speed")}};
}

inline core::WheelSpeedControl get_wheel_speed_control_info(const ros::NodeHandle& nh)
{
  return {
    {load_param<double>(nh, "sensor/speed_std"),
     load_param<double>(nh, "sensor/speed_range")},
    {load_param<double>(nh, "command/maximal_speed"),
     load_param<double>(nh, "command/maximal_acceleration")}};
}

}  // namespace ros1
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_CONTROL_PARAMETERS_HPP_
