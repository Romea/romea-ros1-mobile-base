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

#ifndef ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_GEOMETRY_PARAMETERS_HPP_
#define ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_GEOMETRY_PARAMETERS_HPP_

// ros
#include <ros/ros.h>

// romea
#include <romea_common_utils/params/ros_param.hpp>
#include <romea_core_mobile_base/info/MobileBaseGeometry.hpp>

namespace romea
{
namespace ros1
{

inline core::TrackWheel get_track_wheel_info(const ros::NodeHandle & nh)
{
  auto radius = load_param<double>(nh, "radius");
  auto x = load_param<double>(nh, "x");
  auto z = load_param<double>(nh, "z");
  z = std::isfinite(z) ? z : radius;
  return {radius, x, z};
}

inline core::TrackWheel get_track_sprocket_wheel_info(const ros::NodeHandle & nh)
{
  return get_track_wheel_info(ros::NodeHandle(nh, "sprocket_wheel"));
}

inline std::optional<core::TrackWheel> try_get_track_wheel_info(const ros::NodeHandle & nh)
{
  try {
    return get_track_wheel_info(nh);
  } catch (std::runtime_error & e) {
    std::cout << " " << e.what() << std::endl;
    return {};
  }
}

inline std::vector<core::TrackWheel> get_track_idler_wheels_info(const ros::NodeHandle & nh)
{
  auto idler_wheel = try_get_track_wheel_info(ros::NodeHandle(nh, "idler_wheel"));

  if (idler_wheel) {
    return {*idler_wheel};
  }

  auto front_idler_wheel = try_get_track_wheel_info(ros::NodeHandle(nh, "front_idler_wheel"));
  auto rear_idler_wheel = try_get_track_wheel_info(ros::NodeHandle(nh, "rear_idler_wheel"));

  if (front_idler_wheel && rear_idler_wheel) {
    return {*front_idler_wheel, *rear_idler_wheel};
  }

  return {};
}

inline std::vector<core::TrackWheel> get_track_roller_wheels_info(const ros::NodeHandle & nh)
{
  auto radius = load_param<double>(nh, "roller_wheels/radius");
  auto x_vector = load_vector<double>(nh, "roller_wheels/x");
  auto z = load_param<double>(nh, "roller_wheels/z");
  z = std::isfinite(z) ? z : radius;

  std::vector<core::TrackWheel> roller_wheels;
  for (const double & x : x_vector) {
    core::TrackWheel wheel = {radius, x, z};
    roller_wheels.push_back(wheel);
  }
  return roller_wheels;
}

inline core::Wheel get_wheel_info(const ros::NodeHandle & nh)
{
  return {
    load_param<double>(nh, "radius"),
    load_param<double>(nh, "width"),
    load_param<double>(nh, "hub_carrier_offset")};
}

inline core::ContinuousTrack get_continuous_track_info(const ros::NodeHandle & nh)
{
  return {
    load_param<double>(nh, "width"),
    load_param<double>(nh, "thickness"),
    get_track_sprocket_wheel_info(nh),
    get_track_idler_wheels_info(nh),
    get_track_roller_wheels_info(nh)};
}

inline core::WheeledAxle get_wheeled_axle_info(const ros::NodeHandle & nh)
{
  return {
    load_param<double>(nh, "wheels_distance"),
    get_wheel_info(ros::NodeHandle(nh, "wheels"))};
}

inline core::ContinuousTrackedAxle get_continuous_tracked_axle_info(const ros::NodeHandle & nh)
{
  return {
    load_param<double>(nh, "tracks_distance"),
    get_continuous_track_info(ros::NodeHandle(nh, "tracks"))};
}

inline core::TwoWheeledAxles get_two_wheeled_axles_info(const ros::NodeHandle & nh)
{
  return {
    load_param<double>(nh, "axles_distance"),
    get_wheeled_axle_info(ros::NodeHandle(nh, "front_axle")),
    get_wheeled_axle_info(ros::NodeHandle(nh, "rear_axle"))};
}

}  // namespace ros1
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_GEOMETRY_PARAMETERS_HPP_
