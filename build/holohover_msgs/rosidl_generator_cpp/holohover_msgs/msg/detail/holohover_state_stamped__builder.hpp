// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from holohover_msgs:msg/HolohoverStateStamped.idl
// generated code does not contain a copyright notice

#ifndef HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_STATE_STAMPED__BUILDER_HPP_
#define HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_STATE_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "holohover_msgs/msg/detail/holohover_state_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace holohover_msgs
{

namespace msg
{

namespace builder
{

class Init_HolohoverStateStamped_w_z
{
public:
  explicit Init_HolohoverStateStamped_w_z(::holohover_msgs::msg::HolohoverStateStamped & msg)
  : msg_(msg)
  {}
  ::holohover_msgs::msg::HolohoverStateStamped w_z(::holohover_msgs::msg::HolohoverStateStamped::_w_z_type arg)
  {
    msg_.w_z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::holohover_msgs::msg::HolohoverStateStamped msg_;
};

class Init_HolohoverStateStamped_yaw
{
public:
  explicit Init_HolohoverStateStamped_yaw(::holohover_msgs::msg::HolohoverStateStamped & msg)
  : msg_(msg)
  {}
  Init_HolohoverStateStamped_w_z yaw(::holohover_msgs::msg::HolohoverStateStamped::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_HolohoverStateStamped_w_z(msg_);
  }

private:
  ::holohover_msgs::msg::HolohoverStateStamped msg_;
};

class Init_HolohoverStateStamped_v_y
{
public:
  explicit Init_HolohoverStateStamped_v_y(::holohover_msgs::msg::HolohoverStateStamped & msg)
  : msg_(msg)
  {}
  Init_HolohoverStateStamped_yaw v_y(::holohover_msgs::msg::HolohoverStateStamped::_v_y_type arg)
  {
    msg_.v_y = std::move(arg);
    return Init_HolohoverStateStamped_yaw(msg_);
  }

private:
  ::holohover_msgs::msg::HolohoverStateStamped msg_;
};

class Init_HolohoverStateStamped_v_x
{
public:
  explicit Init_HolohoverStateStamped_v_x(::holohover_msgs::msg::HolohoverStateStamped & msg)
  : msg_(msg)
  {}
  Init_HolohoverStateStamped_v_y v_x(::holohover_msgs::msg::HolohoverStateStamped::_v_x_type arg)
  {
    msg_.v_x = std::move(arg);
    return Init_HolohoverStateStamped_v_y(msg_);
  }

private:
  ::holohover_msgs::msg::HolohoverStateStamped msg_;
};

class Init_HolohoverStateStamped_y
{
public:
  explicit Init_HolohoverStateStamped_y(::holohover_msgs::msg::HolohoverStateStamped & msg)
  : msg_(msg)
  {}
  Init_HolohoverStateStamped_v_x y(::holohover_msgs::msg::HolohoverStateStamped::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_HolohoverStateStamped_v_x(msg_);
  }

private:
  ::holohover_msgs::msg::HolohoverStateStamped msg_;
};

class Init_HolohoverStateStamped_x
{
public:
  explicit Init_HolohoverStateStamped_x(::holohover_msgs::msg::HolohoverStateStamped & msg)
  : msg_(msg)
  {}
  Init_HolohoverStateStamped_y x(::holohover_msgs::msg::HolohoverStateStamped::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_HolohoverStateStamped_y(msg_);
  }

private:
  ::holohover_msgs::msg::HolohoverStateStamped msg_;
};

class Init_HolohoverStateStamped_header
{
public:
  Init_HolohoverStateStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HolohoverStateStamped_x header(::holohover_msgs::msg::HolohoverStateStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_HolohoverStateStamped_x(msg_);
  }

private:
  ::holohover_msgs::msg::HolohoverStateStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::holohover_msgs::msg::HolohoverStateStamped>()
{
  return holohover_msgs::msg::builder::Init_HolohoverStateStamped_header();
}

}  // namespace holohover_msgs

#endif  // HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_STATE_STAMPED__BUILDER_HPP_
