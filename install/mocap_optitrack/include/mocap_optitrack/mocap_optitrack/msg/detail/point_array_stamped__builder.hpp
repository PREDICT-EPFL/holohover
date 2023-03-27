// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mocap_optitrack:msg/PointArrayStamped.idl
// generated code does not contain a copyright notice

#ifndef MOCAP_OPTITRACK__MSG__DETAIL__POINT_ARRAY_STAMPED__BUILDER_HPP_
#define MOCAP_OPTITRACK__MSG__DETAIL__POINT_ARRAY_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mocap_optitrack/msg/detail/point_array_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mocap_optitrack
{

namespace msg
{

namespace builder
{

class Init_PointArrayStamped_points
{
public:
  explicit Init_PointArrayStamped_points(::mocap_optitrack::msg::PointArrayStamped & msg)
  : msg_(msg)
  {}
  ::mocap_optitrack::msg::PointArrayStamped points(::mocap_optitrack::msg::PointArrayStamped::_points_type arg)
  {
    msg_.points = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mocap_optitrack::msg::PointArrayStamped msg_;
};

class Init_PointArrayStamped_header
{
public:
  Init_PointArrayStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PointArrayStamped_points header(::mocap_optitrack::msg::PointArrayStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PointArrayStamped_points(msg_);
  }

private:
  ::mocap_optitrack::msg::PointArrayStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mocap_optitrack::msg::PointArrayStamped>()
{
  return mocap_optitrack::msg::builder::Init_PointArrayStamped_header();
}

}  // namespace mocap_optitrack

#endif  // MOCAP_OPTITRACK__MSG__DETAIL__POINT_ARRAY_STAMPED__BUILDER_HPP_
