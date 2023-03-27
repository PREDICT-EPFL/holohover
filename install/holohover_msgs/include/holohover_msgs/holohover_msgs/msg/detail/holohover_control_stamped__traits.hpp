// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from holohover_msgs:msg/HolohoverControlStamped.idl
// generated code does not contain a copyright notice

#ifndef HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_CONTROL_STAMPED__TRAITS_HPP_
#define HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_CONTROL_STAMPED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "holohover_msgs/msg/detail/holohover_control_stamped__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace holohover_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const HolohoverControlStamped & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: motor_a_1
  {
    out << "motor_a_1: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_a_1, out);
    out << ", ";
  }

  // member: motor_a_2
  {
    out << "motor_a_2: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_a_2, out);
    out << ", ";
  }

  // member: motor_b_1
  {
    out << "motor_b_1: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_b_1, out);
    out << ", ";
  }

  // member: motor_b_2
  {
    out << "motor_b_2: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_b_2, out);
    out << ", ";
  }

  // member: motor_c_1
  {
    out << "motor_c_1: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_c_1, out);
    out << ", ";
  }

  // member: motor_c_2
  {
    out << "motor_c_2: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_c_2, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HolohoverControlStamped & msg,
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

  // member: motor_a_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor_a_1: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_a_1, out);
    out << "\n";
  }

  // member: motor_a_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor_a_2: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_a_2, out);
    out << "\n";
  }

  // member: motor_b_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor_b_1: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_b_1, out);
    out << "\n";
  }

  // member: motor_b_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor_b_2: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_b_2, out);
    out << "\n";
  }

  // member: motor_c_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor_c_1: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_c_1, out);
    out << "\n";
  }

  // member: motor_c_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor_c_2: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_c_2, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HolohoverControlStamped & msg, bool use_flow_style = false)
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
  const holohover_msgs::msg::HolohoverControlStamped & msg,
  std::ostream & out, size_t indentation = 0)
{
  holohover_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use holohover_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const holohover_msgs::msg::HolohoverControlStamped & msg)
{
  return holohover_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<holohover_msgs::msg::HolohoverControlStamped>()
{
  return "holohover_msgs::msg::HolohoverControlStamped";
}

template<>
inline const char * name<holohover_msgs::msg::HolohoverControlStamped>()
{
  return "holohover_msgs/msg/HolohoverControlStamped";
}

template<>
struct has_fixed_size<holohover_msgs::msg::HolohoverControlStamped>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<holohover_msgs::msg::HolohoverControlStamped>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<holohover_msgs::msg::HolohoverControlStamped>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_CONTROL_STAMPED__TRAITS_HPP_
