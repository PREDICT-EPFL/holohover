// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from holohover_msgs:msg/HolohoverIMUStamped.idl
// generated code does not contain a copyright notice

#ifndef HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_IMU_STAMPED__STRUCT_H_
#define HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_IMU_STAMPED__STRUCT_H_

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
// Member 'atti'
#include "holohover_msgs/msg/detail/attitude__struct.h"
// Member 'acc'
// Member 'gyro'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/HolohoverIMUStamped in the package holohover_msgs.
typedef struct holohover_msgs__msg__HolohoverIMUStamped
{
  std_msgs__msg__Header header;
  holohover_msgs__msg__Attitude atti;
  geometry_msgs__msg__Vector3 acc;
  geometry_msgs__msg__Vector3 gyro;
} holohover_msgs__msg__HolohoverIMUStamped;

// Struct for a sequence of holohover_msgs__msg__HolohoverIMUStamped.
typedef struct holohover_msgs__msg__HolohoverIMUStamped__Sequence
{
  holohover_msgs__msg__HolohoverIMUStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} holohover_msgs__msg__HolohoverIMUStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_IMU_STAMPED__STRUCT_H_
