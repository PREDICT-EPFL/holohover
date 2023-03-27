// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rokubimini_msgs:msg/Reading.idl
// generated code does not contain a copyright notice

#ifndef ROKUBIMINI_MSGS__MSG__DETAIL__READING__TRAITS_HPP_
#define ROKUBIMINI_MSGS__MSG__DETAIL__READING__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rokubimini_msgs/msg/detail/reading__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'imu'
// Member 'external_imu'
#include "sensor_msgs/msg/detail/imu__traits.hpp"
// Member 'wrench'
#include "geometry_msgs/msg/detail/wrench_stamped__traits.hpp"
// Member 'temperature'
#include "sensor_msgs/msg/detail/temperature__traits.hpp"

namespace rokubimini_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Reading & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: statusword
  {
    out << "statusword: ";
    rosidl_generator_traits::value_to_yaml(msg.statusword, out);
    out << ", ";
  }

  // member: imu
  {
    out << "imu: ";
    to_flow_style_yaml(msg.imu, out);
    out << ", ";
  }

  // member: wrench
  {
    out << "wrench: ";
    to_flow_style_yaml(msg.wrench, out);
    out << ", ";
  }

  // member: external_imu
  {
    out << "external_imu: ";
    to_flow_style_yaml(msg.external_imu, out);
    out << ", ";
  }

  // member: is_force_torque_saturated
  {
    out << "is_force_torque_saturated: ";
    rosidl_generator_traits::value_to_yaml(msg.is_force_torque_saturated, out);
    out << ", ";
  }

  // member: temperature
  {
    out << "temperature: ";
    to_flow_style_yaml(msg.temperature, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Reading & msg,
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

  // member: statusword
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "statusword: ";
    rosidl_generator_traits::value_to_yaml(msg.statusword, out);
    out << "\n";
  }

  // member: imu
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu:\n";
    to_block_style_yaml(msg.imu, out, indentation + 2);
  }

  // member: wrench
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wrench:\n";
    to_block_style_yaml(msg.wrench, out, indentation + 2);
  }

  // member: external_imu
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "external_imu:\n";
    to_block_style_yaml(msg.external_imu, out, indentation + 2);
  }

  // member: is_force_torque_saturated
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_force_torque_saturated: ";
    rosidl_generator_traits::value_to_yaml(msg.is_force_torque_saturated, out);
    out << "\n";
  }

  // member: temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temperature:\n";
    to_block_style_yaml(msg.temperature, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Reading & msg, bool use_flow_style = false)
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

}  // namespace rokubimini_msgs

namespace rosidl_generator_traits
{

[[deprecated("use rokubimini_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rokubimini_msgs::msg::Reading & msg,
  std::ostream & out, size_t indentation = 0)
{
  rokubimini_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rokubimini_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const rokubimini_msgs::msg::Reading & msg)
{
  return rokubimini_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rokubimini_msgs::msg::Reading>()
{
  return "rokubimini_msgs::msg::Reading";
}

template<>
inline const char * name<rokubimini_msgs::msg::Reading>()
{
  return "rokubimini_msgs/msg/Reading";
}

template<>
struct has_fixed_size<rokubimini_msgs::msg::Reading>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::WrenchStamped>::value && has_fixed_size<sensor_msgs::msg::Imu>::value && has_fixed_size<sensor_msgs::msg::Temperature>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<rokubimini_msgs::msg::Reading>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::WrenchStamped>::value && has_bounded_size<sensor_msgs::msg::Imu>::value && has_bounded_size<sensor_msgs::msg::Temperature>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<rokubimini_msgs::msg::Reading>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROKUBIMINI_MSGS__MSG__DETAIL__READING__TRAITS_HPP_
