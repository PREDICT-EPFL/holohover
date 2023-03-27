// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rokubimini_msgs:srv/GetSensorConfiguration.idl
// generated code does not contain a copyright notice

#ifndef ROKUBIMINI_MSGS__SRV__DETAIL__GET_SENSOR_CONFIGURATION__STRUCT_H_
#define ROKUBIMINI_MSGS__SRV__DETAIL__GET_SENSOR_CONFIGURATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetSensorConfiguration in the package rokubimini_msgs.
typedef struct rokubimini_msgs__srv__GetSensorConfiguration_Request
{
  bool a;
} rokubimini_msgs__srv__GetSensorConfiguration_Request;

// Struct for a sequence of rokubimini_msgs__srv__GetSensorConfiguration_Request.
typedef struct rokubimini_msgs__srv__GetSensorConfiguration_Request__Sequence
{
  rokubimini_msgs__srv__GetSensorConfiguration_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rokubimini_msgs__srv__GetSensorConfiguration_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/GetSensorConfiguration in the package rokubimini_msgs.
typedef struct rokubimini_msgs__srv__GetSensorConfiguration_Response
{
  bool b;
} rokubimini_msgs__srv__GetSensorConfiguration_Response;

// Struct for a sequence of rokubimini_msgs__srv__GetSensorConfiguration_Response.
typedef struct rokubimini_msgs__srv__GetSensorConfiguration_Response__Sequence
{
  rokubimini_msgs__srv__GetSensorConfiguration_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rokubimini_msgs__srv__GetSensorConfiguration_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROKUBIMINI_MSGS__SRV__DETAIL__GET_SENSOR_CONFIGURATION__STRUCT_H_
