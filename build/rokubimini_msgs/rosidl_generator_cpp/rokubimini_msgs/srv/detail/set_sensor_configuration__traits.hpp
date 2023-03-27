// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rokubimini_msgs:srv/SetSensorConfiguration.idl
// generated code does not contain a copyright notice

#ifndef ROKUBIMINI_MSGS__SRV__DETAIL__SET_SENSOR_CONFIGURATION__TRAITS_HPP_
#define ROKUBIMINI_MSGS__SRV__DETAIL__SET_SENSOR_CONFIGURATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rokubimini_msgs/srv/detail/set_sensor_configuration__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rokubimini_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetSensorConfiguration_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: a
  {
    out << "a: ";
    rosidl_generator_traits::value_to_yaml(msg.a, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetSensorConfiguration_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: a
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "a: ";
    rosidl_generator_traits::value_to_yaml(msg.a, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetSensorConfiguration_Request & msg, bool use_flow_style = false)
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
  const rokubimini_msgs::srv::SetSensorConfiguration_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rokubimini_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rokubimini_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rokubimini_msgs::srv::SetSensorConfiguration_Request & msg)
{
  return rokubimini_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rokubimini_msgs::srv::SetSensorConfiguration_Request>()
{
  return "rokubimini_msgs::srv::SetSensorConfiguration_Request";
}

template<>
inline const char * name<rokubimini_msgs::srv::SetSensorConfiguration_Request>()
{
  return "rokubimini_msgs/srv/SetSensorConfiguration_Request";
}

template<>
struct has_fixed_size<rokubimini_msgs::srv::SetSensorConfiguration_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rokubimini_msgs::srv::SetSensorConfiguration_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rokubimini_msgs::srv::SetSensorConfiguration_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rokubimini_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetSensorConfiguration_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: b
  {
    out << "b: ";
    rosidl_generator_traits::value_to_yaml(msg.b, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetSensorConfiguration_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: b
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "b: ";
    rosidl_generator_traits::value_to_yaml(msg.b, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetSensorConfiguration_Response & msg, bool use_flow_style = false)
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
  const rokubimini_msgs::srv::SetSensorConfiguration_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rokubimini_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rokubimini_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rokubimini_msgs::srv::SetSensorConfiguration_Response & msg)
{
  return rokubimini_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rokubimini_msgs::srv::SetSensorConfiguration_Response>()
{
  return "rokubimini_msgs::srv::SetSensorConfiguration_Response";
}

template<>
inline const char * name<rokubimini_msgs::srv::SetSensorConfiguration_Response>()
{
  return "rokubimini_msgs/srv/SetSensorConfiguration_Response";
}

template<>
struct has_fixed_size<rokubimini_msgs::srv::SetSensorConfiguration_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rokubimini_msgs::srv::SetSensorConfiguration_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rokubimini_msgs::srv::SetSensorConfiguration_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rokubimini_msgs::srv::SetSensorConfiguration>()
{
  return "rokubimini_msgs::srv::SetSensorConfiguration";
}

template<>
inline const char * name<rokubimini_msgs::srv::SetSensorConfiguration>()
{
  return "rokubimini_msgs/srv/SetSensorConfiguration";
}

template<>
struct has_fixed_size<rokubimini_msgs::srv::SetSensorConfiguration>
  : std::integral_constant<
    bool,
    has_fixed_size<rokubimini_msgs::srv::SetSensorConfiguration_Request>::value &&
    has_fixed_size<rokubimini_msgs::srv::SetSensorConfiguration_Response>::value
  >
{
};

template<>
struct has_bounded_size<rokubimini_msgs::srv::SetSensorConfiguration>
  : std::integral_constant<
    bool,
    has_bounded_size<rokubimini_msgs::srv::SetSensorConfiguration_Request>::value &&
    has_bounded_size<rokubimini_msgs::srv::SetSensorConfiguration_Response>::value
  >
{
};

template<>
struct is_service<rokubimini_msgs::srv::SetSensorConfiguration>
  : std::true_type
{
};

template<>
struct is_service_request<rokubimini_msgs::srv::SetSensorConfiguration_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rokubimini_msgs::srv::SetSensorConfiguration_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROKUBIMINI_MSGS__SRV__DETAIL__SET_SENSOR_CONFIGURATION__TRAITS_HPP_
