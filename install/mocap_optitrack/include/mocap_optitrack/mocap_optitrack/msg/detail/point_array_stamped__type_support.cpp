// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mocap_optitrack:msg/PointArrayStamped.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mocap_optitrack/msg/detail/point_array_stamped__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace mocap_optitrack
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void PointArrayStamped_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mocap_optitrack::msg::PointArrayStamped(_init);
}

void PointArrayStamped_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mocap_optitrack::msg::PointArrayStamped *>(message_memory);
  typed_message->~PointArrayStamped();
}

size_t size_function__PointArrayStamped__points(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<geometry_msgs::msg::Point> *>(untyped_member);
  return member->size();
}

const void * get_const_function__PointArrayStamped__points(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<geometry_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void * get_function__PointArrayStamped__points(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<geometry_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void fetch_function__PointArrayStamped__points(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const geometry_msgs::msg::Point *>(
    get_const_function__PointArrayStamped__points(untyped_member, index));
  auto & value = *reinterpret_cast<geometry_msgs::msg::Point *>(untyped_value);
  value = item;
}

void assign_function__PointArrayStamped__points(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<geometry_msgs::msg::Point *>(
    get_function__PointArrayStamped__points(untyped_member, index));
  const auto & value = *reinterpret_cast<const geometry_msgs::msg::Point *>(untyped_value);
  item = value;
}

void resize_function__PointArrayStamped__points(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<geometry_msgs::msg::Point> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PointArrayStamped_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mocap_optitrack::msg::PointArrayStamped, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "points",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Point>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mocap_optitrack::msg::PointArrayStamped, points),  // bytes offset in struct
    nullptr,  // default value
    size_function__PointArrayStamped__points,  // size() function pointer
    get_const_function__PointArrayStamped__points,  // get_const(index) function pointer
    get_function__PointArrayStamped__points,  // get(index) function pointer
    fetch_function__PointArrayStamped__points,  // fetch(index, &value) function pointer
    assign_function__PointArrayStamped__points,  // assign(index, value) function pointer
    resize_function__PointArrayStamped__points  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PointArrayStamped_message_members = {
  "mocap_optitrack::msg",  // message namespace
  "PointArrayStamped",  // message name
  2,  // number of fields
  sizeof(mocap_optitrack::msg::PointArrayStamped),
  PointArrayStamped_message_member_array,  // message members
  PointArrayStamped_init_function,  // function to initialize message memory (memory has to be allocated)
  PointArrayStamped_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PointArrayStamped_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PointArrayStamped_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace mocap_optitrack


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mocap_optitrack::msg::PointArrayStamped>()
{
  return &::mocap_optitrack::msg::rosidl_typesupport_introspection_cpp::PointArrayStamped_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mocap_optitrack, msg, PointArrayStamped)() {
  return &::mocap_optitrack::msg::rosidl_typesupport_introspection_cpp::PointArrayStamped_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
