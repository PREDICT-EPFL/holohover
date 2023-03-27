// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rokubimini_msgs:srv/ResetWrench.idl
// generated code does not contain a copyright notice

#ifndef ROKUBIMINI_MSGS__SRV__DETAIL__RESET_WRENCH__TRAITS_HPP_
#define ROKUBIMINI_MSGS__SRV__DETAIL__RESET_WRENCH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rokubimini_msgs/srv/detail/reset_wrench__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'desired_wrench'
#include "geometry_msgs/msg/detail/wrench__traits.hpp"

namespace rokubimini_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ResetWrench_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: desired_wrench
  {
    out << "desired_wrench: ";
    to_flow_style_yaml(msg.desired_wrench, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ResetWrench_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: desired_wrench
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "desired_wrench:\n";
    to_block_style_yaml(msg.desired_wrench, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ResetWrench_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace rokubimini_msgs

namespace rosidl_generator_traits
{

[[deprecated("use rokubimini_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rokubimini_msgs::srv::ResetWrench_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rokubimini_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rokubimini_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rokubimini_msgs::srv::ResetWrench_Request & msg)
{
  return rokubimini_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rokubimini_msgs::srv::ResetWrench_Request>()
{
  return "rokubimini_msgs::srv::ResetWrench_Request";
}

template<>
inline const char * name<rokubimini_msgs::srv::ResetWrench_Request>()
{
  return "rokubimini_msgs/srv/ResetWrench_Request";
}

template<>
struct has_fixed_size<rokubimini_msgs::srv::ResetWrench_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Wrench>::value> {};

template<>
struct has_bounded_size<rokubimini_msgs::srv::ResetWrench_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Wrench>::value> {};

template<>
struct is_message<rokubimini_msgs::srv::ResetWrench_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rokubimini_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ResetWrench_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ResetWrench_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ResetWrench_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace rokubimini_msgs

namespace rosidl_generator_traits
{

[[deprecated("use rokubimini_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rokubimini_msgs::srv::ResetWrench_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rokubimini_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rokubimini_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rokubimini_msgs::srv::ResetWrench_Response & msg)
{
  return rokubimini_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rokubimini_msgs::srv::ResetWrench_Response>()
{
  return "rokubimini_msgs::srv::ResetWrench_Response";
}

template<>
inline const char * name<rokubimini_msgs::srv::ResetWrench_Response>()
{
  return "rokubimini_msgs/srv/ResetWrench_Response";
}

template<>
struct has_fixed_size<rokubimini_msgs::srv::ResetWrench_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rokubimini_msgs::srv::ResetWrench_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rokubimini_msgs::srv::ResetWrench_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rokubimini_msgs::srv::ResetWrench>()
{
  return "rokubimini_msgs::srv::ResetWrench";
}

template<>
inline const char * name<rokubimini_msgs::srv::ResetWrench>()
{
  return "rokubimini_msgs/srv/ResetWrench";
}

template<>
struct has_fixed_size<rokubimini_msgs::srv::ResetWrench>
  : std::integral_constant<
    bool,
    has_fixed_size<rokubimini_msgs::srv::ResetWrench_Request>::value &&
    has_fixed_size<rokubimini_msgs::srv::ResetWrench_Response>::value
  >
{
};

template<>
struct has_bounded_size<rokubimini_msgs::srv::ResetWrench>
  : std::integral_constant<
    bool,
    has_bounded_size<rokubimini_msgs::srv::ResetWrench_Request>::value &&
    has_bounded_size<rokubimini_msgs::srv::ResetWrench_Response>::value
  >
{
};

template<>
struct is_service<rokubimini_msgs::srv::ResetWrench>
  : std::true_type
{
};

template<>
struct is_service_request<rokubimini_msgs::srv::ResetWrench_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rokubimini_msgs::srv::ResetWrench_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROKUBIMINI_MSGS__SRV__DETAIL__RESET_WRENCH__TRAITS_HPP_
