// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from holohover_msgs:msg/HolohoverIMUStamped.idl
// generated code does not contain a copyright notice

#ifndef HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_IMU_STAMPED__TRAITS_HPP_
#define HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_IMU_STAMPED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "holohover_msgs/msg/detail/holohover_imu_stamped__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'atti'
#include "holohover_msgs/msg/detail/attitude__traits.hpp"
// Member 'acc'
// Member 'gyro'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace holohover_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const HolohoverIMUStamped & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: atti
  {
    out << "atti: ";
    to_flow_style_yaml(msg.atti, out);
    out << ", ";
  }

  // member: acc
  {
    out << "acc: ";
    to_flow_style_yaml(msg.acc, out);
    out << ", ";
  }

  // member: gyro
  {
    out << "gyro: ";
    to_flow_style_yaml(msg.gyro, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HolohoverIMUStamped & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: atti
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "atti:\n";
    to_block_style_yaml(msg.atti, out, indentation + 2);
  }

  // member: acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acc:\n";
    to_block_style_yaml(msg.acc, out, indentation + 2);
  }

  // member: gyro
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gyro:\n";
    to_block_style_yaml(msg.gyro, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HolohoverIMUStamped & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace holohover_msgs

namespace rosidl_generator_traits
{

[[deprecated("use holohover_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const holohover_msgs::msg::HolohoverIMUStamped & msg,
  std::ostream & out, size_t indentation = 0)
{
  holohover_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use holohover_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const holohover_msgs::msg::HolohoverIMUStamped & msg)
{
  return holohover_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<holohover_msgs::msg::HolohoverIMUStamped>()
{
  return "holohover_msgs::msg::HolohoverIMUStamped";
}

template<>
inline const char * name<holohover_msgs::msg::HolohoverIMUStamped>()
{
  return "holohover_msgs/msg/HolohoverIMUStamped";
}

template<>
struct has_fixed_size<holohover_msgs::msg::HolohoverIMUStamped>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<holohover_msgs::msg::Attitude>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<holohover_msgs::msg::HolohoverIMUStamped>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<holohover_msgs::msg::Attitude>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<holohover_msgs::msg::HolohoverIMUStamped>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_IMU_STAMPED__TRAITS_HPP_
