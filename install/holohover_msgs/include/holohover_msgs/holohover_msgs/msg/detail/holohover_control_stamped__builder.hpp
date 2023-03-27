// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from holohover_msgs:msg/HolohoverControlStamped.idl
// generated code does not contain a copyright notice

#ifndef HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_CONTROL_STAMPED__BUILDER_HPP_
#define HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_CONTROL_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "holohover_msgs/msg/detail/holohover_control_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace holohover_msgs
{

namespace msg
{

namespace builder
{

class Init_HolohoverControlStamped_motor_c_2
{
public:
  explicit Init_HolohoverControlStamped_motor_c_2(::holohover_msgs::msg::HolohoverControlStamped & msg)
  : msg_(msg)
  {}
  ::holohover_msgs::msg::HolohoverControlStamped motor_c_2(::holohover_msgs::msg::HolohoverControlStamped::_motor_c_2_type arg)
  {
    msg_.motor_c_2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::holohover_msgs::msg::HolohoverControlStamped msg_;
};

class Init_HolohoverControlStamped_motor_c_1
{
public:
  explicit Init_HolohoverControlStamped_motor_c_1(::holohover_msgs::msg::HolohoverControlStamped & msg)
  : msg_(msg)
  {}
  Init_HolohoverControlStamped_motor_c_2 motor_c_1(::holohover_msgs::msg::HolohoverControlStamped::_motor_c_1_type arg)
  {
    msg_.motor_c_1 = std::move(arg);
    return Init_HolohoverControlStamped_motor_c_2(msg_);
  }

private:
  ::holohover_msgs::msg::HolohoverControlStamped msg_;
};

class Init_HolohoverControlStamped_motor_b_2
{
public:
  explicit Init_HolohoverControlStamped_motor_b_2(::holohover_msgs::msg::HolohoverControlStamped & msg)
  : msg_(msg)
  {}
  Init_HolohoverControlStamped_motor_c_1 motor_b_2(::holohover_msgs::msg::HolohoverControlStamped::_motor_b_2_type arg)
  {
    msg_.motor_b_2 = std::move(arg);
    return Init_HolohoverControlStamped_motor_c_1(msg_);
  }

private:
  ::holohover_msgs::msg::HolohoverControlStamped msg_;
};

class Init_HolohoverControlStamped_motor_b_1
{
public:
  explicit Init_HolohoverControlStamped_motor_b_1(::holohover_msgs::msg::HolohoverControlStamped & msg)
  : msg_(msg)
  {}
  Init_HolohoverControlStamped_motor_b_2 motor_b_1(::holohover_msgs::msg::HolohoverControlStamped::_motor_b_1_type arg)
  {
    msg_.motor_b_1 = std::move(arg);
    return Init_HolohoverControlStamped_motor_b_2(msg_);
  }

private:
  ::holohover_msgs::msg::HolohoverControlStamped msg_;
};

class Init_HolohoverControlStamped_motor_a_2
{
public:
  explicit Init_HolohoverControlStamped_motor_a_2(::holohover_msgs::msg::HolohoverControlStamped & msg)
  : msg_(msg)
  {}
  Init_HolohoverControlStamped_motor_b_1 motor_a_2(::holohover_msgs::msg::HolohoverControlStamped::_motor_a_2_type arg)
  {
    msg_.motor_a_2 = std::move(arg);
    return Init_HolohoverControlStamped_motor_b_1(msg_);
  }

private:
  ::holohover_msgs::msg::HolohoverControlStamped msg_;
};

class Init_HolohoverControlStamped_motor_a_1
{
public:
  explicit Init_HolohoverControlStamped_motor_a_1(::holohover_msgs::msg::HolohoverControlStamped & msg)
  : msg_(msg)
  {}
  Init_HolohoverControlStamped_motor_a_2 motor_a_1(::holohover_msgs::msg::HolohoverControlStamped::_motor_a_1_type arg)
  {
    msg_.motor_a_1 = std::move(arg);
    return Init_HolohoverControlStamped_motor_a_2(msg_);
  }

private:
  ::holohover_msgs::msg::HolohoverControlStamped msg_;
};

class Init_HolohoverControlStamped_header
{
public:
  Init_HolohoverControlStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HolohoverControlStamped_motor_a_1 header(::holohover_msgs::msg::HolohoverControlStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_HolohoverControlStamped_motor_a_1(msg_);
  }

private:
  ::holohover_msgs::msg::HolohoverControlStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::holohover_msgs::msg::HolohoverControlStamped>()
{
  return holohover_msgs::msg::builder::Init_HolohoverControlStamped_header();
}

}  // namespace holohover_msgs

#endif  // HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_CONTROL_STAMPED__BUILDER_HPP_
