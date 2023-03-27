// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rokubimini_msgs:srv/FirmwareUpdateEthercat.idl
// generated code does not contain a copyright notice

#ifndef ROKUBIMINI_MSGS__SRV__DETAIL__FIRMWARE_UPDATE_ETHERCAT__TRAITS_HPP_
#define ROKUBIMINI_MSGS__SRV__DETAIL__FIRMWARE_UPDATE_ETHERCAT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rokubimini_msgs/srv/detail/firmware_update_ethercat__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rokubimini_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const FirmwareUpdateEthercat_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: file_name
  {
    out << "file_name: ";
    rosidl_generator_traits::value_to_yaml(msg.file_name, out);
    out << ", ";
  }

  // member: file_path
  {
    out << "file_path: ";
    rosidl_generator_traits::value_to_yaml(msg.file_path, out);
    out << ", ";
  }

  // member: password
  {
    out << "password: ";
    rosidl_generator_traits::value_to_yaml(msg.password, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FirmwareUpdateEthercat_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: file_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "file_name: ";
    rosidl_generator_traits::value_to_yaml(msg.file_name, out);
    out << "\n";
  }

  // member: file_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "file_path: ";
    rosidl_generator_traits::value_to_yaml(msg.file_path, out);
    out << "\n";
  }

  // member: password
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "password: ";
    rosidl_generator_traits::value_to_yaml(msg.password, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FirmwareUpdateEthercat_Request & msg, bool use_flow_style = false)
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
  const rokubimini_msgs::srv::FirmwareUpdateEthercat_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rokubimini_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rokubimini_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rokubimini_msgs::srv::FirmwareUpdateEthercat_Request & msg)
{
  return rokubimini_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rokubimini_msgs::srv::FirmwareUpdateEthercat_Request>()
{
  return "rokubimini_msgs::srv::FirmwareUpdateEthercat_Request";
}

template<>
inline const char * name<rokubimini_msgs::srv::FirmwareUpdateEthercat_Request>()
{
  return "rokubimini_msgs/srv/FirmwareUpdateEthercat_Request";
}

template<>
struct has_fixed_size<rokubimini_msgs::srv::FirmwareUpdateEthercat_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rokubimini_msgs::srv::FirmwareUpdateEthercat_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rokubimini_msgs::srv::FirmwareUpdateEthercat_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rokubimini_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const FirmwareUpdateEthercat_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: result
  {
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FirmwareUpdateEthercat_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FirmwareUpdateEthercat_Response & msg, bool use_flow_style = false)
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
  const rokubimini_msgs::srv::FirmwareUpdateEthercat_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rokubimini_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rokubimini_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rokubimini_msgs::srv::FirmwareUpdateEthercat_Response & msg)
{
  return rokubimini_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rokubimini_msgs::srv::FirmwareUpdateEthercat_Response>()
{
  return "rokubimini_msgs::srv::FirmwareUpdateEthercat_Response";
}

template<>
inline const char * name<rokubimini_msgs::srv::FirmwareUpdateEthercat_Response>()
{
  return "rokubimini_msgs/srv/FirmwareUpdateEthercat_Response";
}

template<>
struct has_fixed_size<rokubimini_msgs::srv::FirmwareUpdateEthercat_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rokubimini_msgs::srv::FirmwareUpdateEthercat_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rokubimini_msgs::srv::FirmwareUpdateEthercat_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rokubimini_msgs::srv::FirmwareUpdateEthercat>()
{
  return "rokubimini_msgs::srv::FirmwareUpdateEthercat";
}

template<>
inline const char * name<rokubimini_msgs::srv::FirmwareUpdateEthercat>()
{
  return "rokubimini_msgs/srv/FirmwareUpdateEthercat";
}

template<>
struct has_fixed_size<rokubimini_msgs::srv::FirmwareUpdateEthercat>
  : std::integral_constant<
    bool,
    has_fixed_size<rokubimini_msgs::srv::FirmwareUpdateEthercat_Request>::value &&
    has_fixed_size<rokubimini_msgs::srv::FirmwareUpdateEthercat_Response>::value
  >
{
};

template<>
struct has_bounded_size<rokubimini_msgs::srv::FirmwareUpdateEthercat>
  : std::integral_constant<
    bool,
    has_bounded_size<rokubimini_msgs::srv::FirmwareUpdateEthercat_Request>::value &&
    has_bounded_size<rokubimini_msgs::srv::FirmwareUpdateEthercat_Response>::value
  >
{
};

template<>
struct is_service<rokubimini_msgs::srv::FirmwareUpdateEthercat>
  : std::true_type
{
};

template<>
struct is_service_request<rokubimini_msgs::srv::FirmwareUpdateEthercat_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rokubimini_msgs::srv::FirmwareUpdateEthercat_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROKUBIMINI_MSGS__SRV__DETAIL__FIRMWARE_UPDATE_ETHERCAT__TRAITS_HPP_
