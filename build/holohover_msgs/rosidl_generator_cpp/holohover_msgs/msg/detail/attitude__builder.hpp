// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from holohover_msgs:msg/Attitude.idl
// generated code does not contain a copyright notice

#ifndef HOLOHOVER_MSGS__MSG__DETAIL__ATTITUDE__BUILDER_HPP_
#define HOLOHOVER_MSGS__MSG__DETAIL__ATTITUDE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "holohover_msgs/msg/detail/attitude__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace holohover_msgs
{

namespace msg
{

namespace builder
{

class Init_Attitude_yaw
{
public:
  explicit Init_Attitude_yaw(::holohover_msgs::msg::Attitude & msg)
  : msg_(msg)
  {}
  ::holohover_msgs::msg::Attitude yaw(::holohover_msgs::msg::Attitude::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::holohover_msgs::msg::Attitude msg_;
};

class Init_Attitude_pitch
{
public:
  explicit Init_Attitude_pitch(::holohover_msgs::msg::Attitude & msg)
  : msg_(msg)
  {}
  Init_Attitude_yaw pitch(::holohover_msgs::msg::Attitude::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_Attitude_yaw(msg_);
  }

private:
  ::holohover_msgs::msg::Attitude msg_;
};

class Init_Attitude_roll
{
public:
  Init_Attitude_roll()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Attitude_pitch roll(::holohover_msgs::msg::Attitude::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_Attitude_pitch(msg_);
  }

private:
  ::holohover_msgs::msg::Attitude msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::holohover_msgs::msg::Attitude>()
{
  return holohover_msgs::msg::builder::Init_Attitude_roll();
}

}  // namespace holohover_msgs

#endif  // HOLOHOVER_MSGS__MSG__DETAIL__ATTITUDE__BUILDER_HPP_
