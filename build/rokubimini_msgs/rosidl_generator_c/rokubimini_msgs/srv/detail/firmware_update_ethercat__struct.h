// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rokubimini_msgs:srv/FirmwareUpdateEthercat.idl
// generated code does not contain a copyright notice

#ifndef ROKUBIMINI_MSGS__SRV__DETAIL__FIRMWARE_UPDATE_ETHERCAT__STRUCT_H_
#define ROKUBIMINI_MSGS__SRV__DETAIL__FIRMWARE_UPDATE_ETHERCAT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'file_name'
// Member 'file_path'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/FirmwareUpdateEthercat in the package rokubimini_msgs.
typedef struct rokubimini_msgs__srv__FirmwareUpdateEthercat_Request
{
  rosidl_runtime_c__String file_name;
  rosidl_runtime_c__String file_path;
  uint32_t password;
} rokubimini_msgs__srv__FirmwareUpdateEthercat_Request;

// Struct for a sequence of rokubimini_msgs__srv__FirmwareUpdateEthercat_Request.
typedef struct rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__Sequence
{
  rokubimini_msgs__srv__FirmwareUpdateEthercat_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/FirmwareUpdateEthercat in the package rokubimini_msgs.
typedef struct rokubimini_msgs__srv__FirmwareUpdateEthercat_Response
{
  bool result;
} rokubimini_msgs__srv__FirmwareUpdateEthercat_Response;

// Struct for a sequence of rokubimini_msgs__srv__FirmwareUpdateEthercat_Response.
typedef struct rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__Sequence
{
  rokubimini_msgs__srv__FirmwareUpdateEthercat_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROKUBIMINI_MSGS__SRV__DETAIL__FIRMWARE_UPDATE_ETHERCAT__STRUCT_H_
