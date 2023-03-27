// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from holohover_msgs:msg/HolohoverMouseStamped.idl
// generated code does not contain a copyright notice

#ifndef HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_MOUSE_STAMPED__BUILDER_HPP_
#define HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_MOUSE_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "holohover_msgs/msg/detail/holohover_mouse_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace holohover_msgs
{

namespace msg
{

namespace builder
{

class Init_HolohoverMouseStamped_v_y
{
public:
  explicit Init_HolohoverMouseStamped_v_y(::holohover_msgs::msg::HolohoverMouseStamped & msg)
  : msg_(msg)
  {}
  ::holohover_msgs::msg::HolohoverMouseStamped v_y(::holohover_msgs::msg::HolohoverMouseStamped::_v_y_type arg)
  {
    msg_.v_y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::holohover_msgs::msg::HolohoverMouseStamped msg_;
};

class Init_HolohoverMouseStamped_v_x
{
public:
  explicit Init_HolohoverMouseStamped_v_x(::holohover_msgs::msg::HolohoverMouseStamped & msg)
  : msg_(msg)
  {}
  Init_HolohoverMouseStamped_v_y v_x(::holohover_msgs::msg::HolohoverMouseStamped::_v_x_type arg)
  {
    msg_.v_x = std::move(arg);
    return Init_HolohoverMouseStamped_v_y(msg_);
  }

private:
  ::holohover_msgs::msg::HolohoverMouseStamped msg_;
};

class Init_HolohoverMouseStamped_header
{
public:
  Init_HolohoverMouseStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HolohoverMouseStamped_v_x header(::holohover_msgs::msg::HolohoverMouseStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_HolohoverMouseStamped_v_x(msg_);
  }

private:
  ::holohover_msgs::msg::HolohoverMouseStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::holohover_msgs::msg::HolohoverMouseStamped>()
{
  return holohover_msgs::msg::builder::Init_HolohoverMouseStamped_header();
}

}  // namespace holohover_msgs

#endif  // HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_MOUSE_STAMPED__BUILDER_HPP_
