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

#ifndef ROMEA_MOBILE_BASE_UTILS__CONTROL__COMMAND_PUBLISHER_HPP_
#define ROMEA_MOBILE_BASE_UTILS__CONTROL__COMMAND_PUBLISHER_HPP_

// std
#include <memory>
#include <string>

// local
#include "romea_mobile_base_utils/conversions/command_conversions.hpp"

// romea
#include <romea_common_utils/publishers/data_publisher.hpp>

namespace romea
{
namespace ros1
{

template<typename CommandType, typename MsgType>
std::unique_ptr<DataPublisher<CommandType, MsgType>> make_command_publisher(
  ros::NodeHandle & nh, const std::string & topic_name)
{
  return make_data_publisher<CommandType, MsgType>(nh, topic_name, 1, false);
}

template<typename CommandType>
struct CommandPublisher
{
};

template<>
struct CommandPublisher<core::SkidSteeringCommand>
{
  using PubType = PublisherBase<core::SkidSteeringCommand>;

  template<typename MsgType>
  static std::unique_ptr<PubType> instance(ros::NodeHandle & nh, const std::string & topic_name)
  {
    return make_command_publisher<core::SkidSteeringCommand, MsgType>(nh, topic_name);
  }

  static std::unique_ptr<PubType> instance(ros::NodeHandle & nh, const std::string & message_type)
  {
    if (message_type == "geometry_msgs/Twist") {
      using msg = geometry_msgs::Twist;
      return instance<msg>(nh, "cmd_vel");
    } else if (message_type == "romea_mobile_base_msgs/SkidSteeringCommand") {
      using msg = romea_mobile_base_msgs::SkidSteeringCommand;
      return instance<msg>(nh, "cmd_skid_steering");
    } else {
      throw std::runtime_error(
        "Output message type " + message_type +
        " is unsupported by skid steering command publisher");
    }
  }
};

template<>
struct CommandPublisher<core::OmniSteeringCommand>
{
  using PubType = PublisherBase<core::OmniSteeringCommand>;

  template<typename MsgType>
  static std::unique_ptr<PubType> instance(ros::NodeHandle & nh, const std::string & topic_name)
  {
    return make_command_publisher<core::OmniSteeringCommand, MsgType>(nh, topic_name);
  }

  static std::unique_ptr<PubType> instance(ros::NodeHandle & nh, const std::string & message_type)
  {
    if (message_type == "geometry_msgs/Twist") {
      using msg = geometry_msgs::Twist;
      return instance<msg>(nh, "cmd_vel");
    } else if (message_type == "romea_mobile_base_msgs/OmniSteeringCommand") {
      using msg = romea_mobile_base_msgs::OmniSteeringCommand;
      return instance<msg>(nh, "cmd_omni_steering");
    } else {
      throw std::runtime_error(
        "Output message type " + message_type +
        " is unsupported by omni steering command publisher");
    }
  }
};

template<>
struct CommandPublisher<core::OneAxleSteeringCommand>
{
  using PubType = PublisherBase<core::OneAxleSteeringCommand>;

  template<typename MsgType>
  static std::unique_ptr<PubType> instance(ros::NodeHandle & nh, const std::string & topic_name)
  {
    return make_command_publisher<core::OneAxleSteeringCommand, MsgType>(nh, topic_name);
  }

  static std::unique_ptr<PubType> instance(ros::NodeHandle & nh, const std::string & message_type)
  {
    if (message_type == "geometry_msgs/Twist") {
      using msg = geometry_msgs::Twist;
      return instance<msg>(nh, "cmd_vel");
    } else if (message_type == "ackermann_msgs/AckermannDrive") {
      using msg = ackermann_msgs::AckermannDrive;
      return instance<msg>(nh, "cmd_steer");
    } else if (message_type == "romea_mobile_base_msgs/OneAxleSteeringCommand") {
      using msg = romea_mobile_base_msgs::OneAxleSteeringCommand;
      return instance<msg>(nh, "cmd_one_axle_steering");
    } else {
      throw std::runtime_error(
        "Output message type " + message_type +
        " is unsupported by one axle steering command publisher");
    }
  }
};

template<>
struct CommandPublisher<core::TwoAxleSteeringCommand>
{
  using PubType = PublisherBase<core::TwoAxleSteeringCommand>;

  template<typename MsgType>
  static std::unique_ptr<PubType> instance(ros::NodeHandle & nh, const std::string & topic_name)
  {
    return make_command_publisher<core::TwoAxleSteeringCommand, MsgType>(nh, topic_name);
  }

  static std::unique_ptr<PubType> instance(ros::NodeHandle & nh, const std::string & message_type)
  {
    if (message_type == "four_wheel_steering_msgs/FourWheelSteering") {
      using msg = four_wheel_steering_msgs::FourWheelSteering;
      return instance<msg>(nh, "cmd_4ws");
    } else if (message_type == "romea_mobile_base_msgs/TwoAxleSteeringCommand") {
      using msg = romea_mobile_base_msgs::TwoAxleSteeringCommand;
      return instance<msg>(nh, "cmd_two_axle_steering");
    } else {
      throw std::runtime_error(
        "Output message type " + message_type +
        " is unsupported by two axle steering command publisher");
    }
  }
};

template<typename CommandType>
std::unique_ptr<PublisherBase<CommandType>> make_command_publisher(
  ros::NodeHandle & nh, const std::string message_type)
{
  if constexpr (std::is_same_v<CommandType, core::SkidSteeringCommand>) {
    return CommandPublisher<core::SkidSteeringCommand>::instance(nh, message_type);
  }

  if constexpr (std::is_same_v<CommandType, core::OmniSteeringCommand>) {
    return CommandPublisher<core::OmniSteeringCommand>::instance(nh, message_type);
  }

  if constexpr (std::is_same_v<CommandType, core::OneAxleSteeringCommand>) {
    return CommandPublisher<core::OneAxleSteeringCommand>::instance(nh, message_type);
  }

  if constexpr (std::is_same_v<CommandType, core::TwoAxleSteeringCommand>) {
    return CommandPublisher<core::TwoAxleSteeringCommand>::instance(nh, message_type);
  }
}

}  // namespace ros1
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__CONTROL__COMMAND_PUBLISHER_HPP_
