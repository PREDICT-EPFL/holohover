// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from holohover_msgs:msg/HolohoverMouseStamped.idl
// generated code does not contain a copyright notice

#ifndef HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_MOUSE_STAMPED__STRUCT_H_
#define HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_MOUSE_STAMPED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/HolohoverMouseStamped in the package holohover_msgs.
typedef struct holohover_msgs__msg__HolohoverMouseStamped
{
  std_msgs__msg__Header header;
  double v_x;
  double v_y;
} holohover_msgs__msg__HolohoverMouseStamped;

// Struct for a sequence of holohover_msgs__msg__HolohoverMouseStamped.
typedef struct holohover_msgs__msg__HolohoverMouseStamped__Sequence
{
  holohover_msgs__msg__HolohoverMouseStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} holohover_msgs__msg__HolohoverMouseStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_MOUSE_STAMPED__STRUCT_H_
