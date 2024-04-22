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

#ifndef ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_PARAMETERS2FWS2RWD_HPP_
#define ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_PARAMETERS2FWS2RWD_HPP_

// ros
#include <ros/node_handle.h>

// romea
#include "romea_core_mobile_base/info/MobileBaseInfo2FWS2RWD.hpp"
#include "romea_mobile_base_utils/params/mobile_base_control_parameters.hpp"
#include "romea_mobile_base_utils/params/mobile_base_geometry_parameters.hpp"
#include "romea_mobile_base_utils/params/mobile_base_inertia_parameters.hpp"

namespace romea
{
namespace ros1
{

inline core::MobileBaseInfo2FWS2RWD get_mobile_base_info_2FWS2RWD(const ros::NodeHandle & nh)
{
  return {
    get_two_wheeled_axles_info(ros::NodeHandle(nh, "geometry")),
    get_steering_angle_control_info(ros::NodeHandle(nh, "front_wheels_steering_control")),
    get_wheel_speed_control_info(ros::NodeHandle(nh, "rear_wheels_speed_control")),
    get_inertia_info(ros::NodeHandle(nh, "inertia")),
    loadEigenVector<Eigen::Vector3d>(nh, "control_point")};
}

}  // namespace ros1
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_PARAMETERS2FWS2RWD_HPP_
