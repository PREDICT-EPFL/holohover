// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from holohover_msgs:msg/HolohoverStateStamped.idl
// generated code does not contain a copyright notice

#ifndef HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_STATE_STAMPED__TRAITS_HPP_
#define HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_STATE_STAMPED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "holohover_msgs/msg/detail/holohover_state_stamped__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace holohover_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const HolohoverStateStamped & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: v_x
  {
    out << "v_x: ";
    rosidl_generator_traits::value_to_yaml(msg.v_x, out);
    out << ", ";
  }

  // member: v_y
  {
    out << "v_y: ";
    rosidl_generator_traits::value_to_yaml(msg.v_y, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: w_z
  {
    out << "w_z: ";
    rosidl_generator_traits::value_to_yaml(msg.w_z, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HolohoverStateStamped & msg,
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

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: v_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "v_x: ";
    rosidl_generator_traits::value_to_yaml(msg.v_x, out);
    out << "\n";
  }

  // member: v_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "v_y: ";
    rosidl_generator_traits::value_to_yaml(msg.v_y, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }

  // member: w_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "w_z: ";
    rosidl_generator_traits::value_to_yaml(msg.w_z, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HolohoverStateStamped & msg, bool use_flow_style = false)
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
  const holohover_msgs::msg::HolohoverStateStamped & msg,
  std::ostream & out, size_t indentation = 0)
{
  holohover_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use holohover_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const holohover_msgs::msg::HolohoverStateStamped & msg)
{
  return holohover_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<holohover_msgs::msg::HolohoverStateStamped>()
{
  return "holohover_msgs::msg::HolohoverStateStamped";
}

template<>
inline const char * name<holohover_msgs::msg::HolohoverStateStamped>()
{
  return "holohover_msgs/msg/HolohoverStateStamped";
}

template<>
struct has_fixed_size<holohover_msgs::msg::HolohoverStateStamped>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<holohover_msgs::msg::HolohoverStateStamped>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<holohover_msgs::msg::HolohoverStateStamped>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_STATE_STAMPED__TRAITS_HPP_
