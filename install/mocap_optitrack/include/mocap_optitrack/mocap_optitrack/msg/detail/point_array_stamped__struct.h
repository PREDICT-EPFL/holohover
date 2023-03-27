// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mocap_optitrack:msg/PointArrayStamped.idl
// generated code does not contain a copyright notice

#ifndef MOCAP_OPTITRACK__MSG__DETAIL__POINT_ARRAY_STAMPED__STRUCT_H_
#define MOCAP_OPTITRACK__MSG__DETAIL__POINT_ARRAY_STAMPED__STRUCT_H_

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
// Member 'points'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/PointArrayStamped in the package mocap_optitrack.
/**
  * An array of points with a header
 */
typedef struct mocap_optitrack__msg__PointArrayStamped
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Point__Sequence points;
} mocap_optitrack__msg__PointArrayStamped;

// Struct for a sequence of mocap_optitrack__msg__PointArrayStamped.
typedef struct mocap_optitrack__msg__PointArrayStamped__Sequence
{
  mocap_optitrack__msg__PointArrayStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mocap_optitrack__msg__PointArrayStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOCAP_OPTITRACK__MSG__DETAIL__POINT_ARRAY_STAMPED__STRUCT_H_
