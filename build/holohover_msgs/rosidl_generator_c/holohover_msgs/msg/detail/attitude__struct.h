// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from holohover_msgs:msg/Attitude.idl
// generated code does not contain a copyright notice

#ifndef HOLOHOVER_MSGS__MSG__DETAIL__ATTITUDE__STRUCT_H_
#define HOLOHOVER_MSGS__MSG__DETAIL__ATTITUDE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Attitude in the package holohover_msgs.
typedef struct holohover_msgs__msg__Attitude
{
  double roll;
  double pitch;
  double yaw;
} holohover_msgs__msg__Attitude;

// Struct for a sequence of holohover_msgs__msg__Attitude.
typedef struct holohover_msgs__msg__Attitude__Sequence
{
  holohover_msgs__msg__Attitude * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} holohover_msgs__msg__Attitude__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HOLOHOVER_MSGS__MSG__DETAIL__ATTITUDE__STRUCT_H_
