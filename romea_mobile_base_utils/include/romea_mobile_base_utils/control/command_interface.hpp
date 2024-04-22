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

#ifndef ROMEA_MOBILE_BASE_UTILS__CONTROL__COMMAND_INTERFACE_HPP_
#define ROMEA_MOBILE_BASE_UTILS__CONTROL__COMMAND_INTERFACE_HPP_

// std
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

// romea
// #include "romea_cmd_mux_utils/cmd_mux_interface.hpp"

#include "romea_mobile_base_utils/control/command_publisher.hpp"
#include "romea_mobile_base_utils/conversions/kinematic_conversions.hpp"

namespace romea
{
namespace ros1
{

struct CommandInterfaceConfiguration
{
  std::string output_message_type;
  int priority;
  double rate;
};

template<typename CommandType>
class CommandInterface
{
public:
  using CmdPublisher = PublisherBase<CommandType>;
  using Configuration = CommandInterfaceConfiguration;

public:
  CommandInterface(ros::NodeHandle & nh, const Configuration & configuration);

  void send_null_command();

  void send_command(const CommandType & command);

  void connect_timeout_callback(std::function<void(void)> callback);

public:
  void start();

  void stop(bool reset);

  void enable_emergency_stop();

  void disable_emergency_stop();

public:
  bool is_started();

  bool is_emergency_stop_activated();

private:
  void timer_callback_();

  void publish_command_(const bool & timeout);

  // void subscribe_to_cmd_mux(const int & priority, const double & timetout);

private:
  std::unique_ptr<CmdPublisher> cmd_pub_;
  // CmdMuxInterface cmd_mux_client_;

  std::atomic<bool> is_started_;
  std::atomic<bool> is_emergency_stop_activated_;

  ros::Timer timer_;
  ros::Duration timeout_duration_;
  ros::Time last_command_date_;
  CommandType command_;

  std::mutex mutex_;

  std::function<void(void)> timeout_callback_;
};

//-----------------------------------------------------------------------------
template<typename CommandType>
CommandInterface<CommandType>::CommandInterface(
  ros::NodeHandle & nh, const Configuration & configuration)
: cmd_pub_(make_command_publisher<CommandType>(nh, configuration.output_message_type)),
  // cmd_mux_client_(node),
  is_started_(false),
  is_emergency_stop_activated_(false),
  timeout_duration_(0, 0),
  last_command_date_(ros::Time::now())
{
  // const int & priority = configuration.priority;
  double period = 1 / configuration.rate;
  double timeout = 2 * period;

  timeout_duration_ = ros::Duration(timeout);
  // subscribe_to_cmd_mux(priority, timeout);

  boost::function<void(const ros::TimerEvent &)> timer_cb = [this](auto) { timer_callback_(); };
  timer_ = nh.createTimer(ros::Duration(period), timer_cb);
}

}  // namespace ros1
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__CONTROL__COMMAND_INTERFACE_HPP_
