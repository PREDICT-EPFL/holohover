// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mocap_optitrack:msg/PointArrayStamped.idl
// generated code does not contain a copyright notice

#ifndef MOCAP_OPTITRACK__MSG__DETAIL__POINT_ARRAY_STAMPED__TRAITS_HPP_
#define MOCAP_OPTITRACK__MSG__DETAIL__POINT_ARRAY_STAMPED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mocap_optitrack/msg/detail/point_array_stamped__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'points'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace mocap_optitrack
{

namespace msg
{

inline void to_flow_style_yaml(
  const PointArrayStamped & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: points
  {
    if (msg.points.size() == 0) {
      out << "points: []";
    } else {
      out << "points: [";
      size_t pending_items = msg.points.size();
      for (auto item : msg.points) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PointArrayStamped & msg,
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

  // member: points
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.points.size() == 0) {
      out << "points: []\n";
    } else {
      out << "points:\n";
      for (auto item : msg.points) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PointArrayStamped & msg, bool use_flow_style = false)
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

}  // namespace mocap_optitrack

namespace rosidl_generator_traits
{

[[deprecated("use mocap_optitrack::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mocap_optitrack::msg::PointArrayStamped & msg,
  std::ostream & out, size_t indentation = 0)
{
  mocap_optitrack::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mocap_optitrack::msg::to_yaml() instead")]]
inline std::string to_yaml(const mocap_optitrack::msg::PointArrayStamped & msg)
{
  return mocap_optitrack::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mocap_optitrack::msg::PointArrayStamped>()
{
  return "mocap_optitrack::msg::PointArrayStamped";
}

template<>
inline const char * name<mocap_optitrack::msg::PointArrayStamped>()
{
  return "mocap_optitrack/msg/PointArrayStamped";
}

template<>
struct has_fixed_size<mocap_optitrack::msg::PointArrayStamped>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mocap_optitrack::msg::PointArrayStamped>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mocap_optitrack::msg::PointArrayStamped>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MOCAP_OPTITRACK__MSG__DETAIL__POINT_ARRAY_STAMPED__TRAITS_HPP_
