// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rokubimini_msgs:srv/ResetWrench.idl
// generated code does not contain a copyright notice

#ifndef ROKUBIMINI_MSGS__SRV__DETAIL__RESET_WRENCH__STRUCT_H_
#define ROKUBIMINI_MSGS__SRV__DETAIL__RESET_WRENCH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'desired_wrench'
#include "geometry_msgs/msg/detail/wrench__struct.h"

/// Struct defined in srv/ResetWrench in the package rokubimini_msgs.
typedef struct rokubimini_msgs__srv__ResetWrench_Request
{
  geometry_msgs__msg__Wrench desired_wrench;
} rokubimini_msgs__srv__ResetWrench_Request;

// Struct for a sequence of rokubimini_msgs__srv__ResetWrench_Request.
typedef struct rokubimini_msgs__srv__ResetWrench_Request__Sequence
{
  rokubimini_msgs__srv__ResetWrench_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rokubimini_msgs__srv__ResetWrench_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/ResetWrench in the package rokubimini_msgs.
typedef struct rokubimini_msgs__srv__ResetWrench_Response
{
  bool success;
} rokubimini_msgs__srv__ResetWrench_Response;

// Struct for a sequence of rokubimini_msgs__srv__ResetWrench_Response.
typedef struct rokubimini_msgs__srv__ResetWrench_Response__Sequence
{
  rokubimini_msgs__srv__ResetWrench_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rokubimini_msgs__srv__ResetWrench_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROKUBIMINI_MSGS__SRV__DETAIL__RESET_WRENCH__STRUCT_H_
