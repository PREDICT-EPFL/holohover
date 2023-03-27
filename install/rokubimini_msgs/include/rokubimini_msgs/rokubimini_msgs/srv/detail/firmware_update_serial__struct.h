// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rokubimini_msgs:srv/FirmwareUpdateSerial.idl
// generated code does not contain a copyright notice

#ifndef ROKUBIMINI_MSGS__SRV__DETAIL__FIRMWARE_UPDATE_SERIAL__STRUCT_H_
#define ROKUBIMINI_MSGS__SRV__DETAIL__FIRMWARE_UPDATE_SERIAL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'file_path'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/FirmwareUpdateSerial in the package rokubimini_msgs.
typedef struct rokubimini_msgs__srv__FirmwareUpdateSerial_Request
{
  rosidl_runtime_c__String file_path;
} rokubimini_msgs__srv__FirmwareUpdateSerial_Request;

// Struct for a sequence of rokubimini_msgs__srv__FirmwareUpdateSerial_Request.
typedef struct rokubimini_msgs__srv__FirmwareUpdateSerial_Request__Sequence
{
  rokubimini_msgs__srv__FirmwareUpdateSerial_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rokubimini_msgs__srv__FirmwareUpdateSerial_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/FirmwareUpdateSerial in the package rokubimini_msgs.
typedef struct rokubimini_msgs__srv__FirmwareUpdateSerial_Response
{
  bool result;
} rokubimini_msgs__srv__FirmwareUpdateSerial_Response;

// Struct for a sequence of rokubimini_msgs__srv__FirmwareUpdateSerial_Response.
typedef struct rokubimini_msgs__srv__FirmwareUpdateSerial_Response__Sequence
{
  rokubimini_msgs__srv__FirmwareUpdateSerial_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rokubimini_msgs__srv__FirmwareUpdateSerial_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROKUBIMINI_MSGS__SRV__DETAIL__FIRMWARE_UPDATE_SERIAL__STRUCT_H_
