// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rokubimini_msgs:msg/Reading.idl
// generated code does not contain a copyright notice

#ifndef ROKUBIMINI_MSGS__MSG__DETAIL__READING__BUILDER_HPP_
#define ROKUBIMINI_MSGS__MSG__DETAIL__READING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rokubimini_msgs/msg/detail/reading__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rokubimini_msgs
{

namespace msg
{

namespace builder
{

class Init_Reading_temperature
{
public:
  explicit Init_Reading_temperature(::rokubimini_msgs::msg::Reading & msg)
  : msg_(msg)
  {}
  ::rokubimini_msgs::msg::Reading temperature(::rokubimini_msgs::msg::Reading::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rokubimini_msgs::msg::Reading msg_;
};

class Init_Reading_is_force_torque_saturated
{
public:
  explicit Init_Reading_is_force_torque_saturated(::rokubimini_msgs::msg::Reading & msg)
  : msg_(msg)
  {}
  Init_Reading_temperature is_force_torque_saturated(::rokubimini_msgs::msg::Reading::_is_force_torque_saturated_type arg)
  {
    msg_.is_force_torque_saturated = std::move(arg);
    return Init_Reading_temperature(msg_);
  }

private:
  ::rokubimini_msgs::msg::Reading msg_;
};

class Init_Reading_external_imu
{
public:
  explicit Init_Reading_external_imu(::rokubimini_msgs::msg::Reading & msg)
  : msg_(msg)
  {}
  Init_Reading_is_force_torque_saturated external_imu(::rokubimini_msgs::msg::Reading::_external_imu_type arg)
  {
    msg_.external_imu = std::move(arg);
    return Init_Reading_is_force_torque_saturated(msg_);
  }

private:
  ::rokubimini_msgs::msg::Reading msg_;
};

class Init_Reading_wrench
{
public:
  explicit Init_Reading_wrench(::rokubimini_msgs::msg::Reading & msg)
  : msg_(msg)
  {}
  Init_Reading_external_imu wrench(::rokubimini_msgs::msg::Reading::_wrench_type arg)
  {
    msg_.wrench = std::move(arg);
    return Init_Reading_external_imu(msg_);
  }

private:
  ::rokubimini_msgs::msg::Reading msg_;
};

class Init_Reading_imu
{
public:
  explicit Init_Reading_imu(::rokubimini_msgs::msg::Reading & msg)
  : msg_(msg)
  {}
  Init_Reading_wrench imu(::rokubimini_msgs::msg::Reading::_imu_type arg)
  {
    msg_.imu = std::move(arg);
    return Init_Reading_wrench(msg_);
  }

private:
  ::rokubimini_msgs::msg::Reading msg_;
};

class Init_Reading_statusword
{
public:
  explicit Init_Reading_statusword(::rokubimini_msgs::msg::Reading & msg)
  : msg_(msg)
  {}
  Init_Reading_imu statusword(::rokubimini_msgs::msg::Reading::_statusword_type arg)
  {
    msg_.statusword = std::move(arg);
    return Init_Reading_imu(msg_);
  }

private:
  ::rokubimini_msgs::msg::Reading msg_;
};

class Init_Reading_header
{
public:
  Init_Reading_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Reading_statusword header(::rokubimini_msgs::msg::Reading::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Reading_statusword(msg_);
  }

private:
  ::rokubimini_msgs::msg::Reading msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rokubimini_msgs::msg::Reading>()
{
  return rokubimini_msgs::msg::builder::Init_Reading_header();
}

}  // namespace rokubimini_msgs

#endif  // ROKUBIMINI_MSGS__MSG__DETAIL__READING__BUILDER_HPP_
