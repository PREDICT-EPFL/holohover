// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from holohover_msgs:msg/HolohoverStateStamped.idl
// generated code does not contain a copyright notice

#ifndef HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_STATE_STAMPED__STRUCT_H_
#define HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_STATE_STAMPED__STRUCT_H_

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

/// Struct defined in msg/HolohoverStateStamped in the package holohover_msgs.
typedef struct holohover_msgs__msg__HolohoverStateStamped
{
  std_msgs__msg__Header header;
  double x;
  double y;
  double v_x;
  double v_y;
  double yaw;
  double w_z;
} holohover_msgs__msg__HolohoverStateStamped;

// Struct for a sequence of holohover_msgs__msg__HolohoverStateStamped.
typedef struct holohover_msgs__msg__HolohoverStateStamped__Sequence
{
  holohover_msgs__msg__HolohoverStateStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} holohover_msgs__msg__HolohoverStateStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_STATE_STAMPED__STRUCT_H_
