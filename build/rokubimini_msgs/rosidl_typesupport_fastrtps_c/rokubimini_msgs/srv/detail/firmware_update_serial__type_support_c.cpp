// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from rokubimini_msgs:srv/FirmwareUpdateSerial.idl
// generated code does not contain a copyright notice
#include "rokubimini_msgs/srv/detail/firmware_update_serial__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rokubimini_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rokubimini_msgs/srv/detail/firmware_update_serial__struct.h"
#include "rokubimini_msgs/srv/detail/firmware_update_serial__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // file_path
#include "rosidl_runtime_c/string_functions.h"  // file_path

// forward declare type support functions


using _FirmwareUpdateSerial_Request__ros_msg_type = rokubimini_msgs__srv__FirmwareUpdateSerial_Request;

static bool _FirmwareUpdateSerial_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _FirmwareUpdateSerial_Request__ros_msg_type * ros_message = static_cast<const _FirmwareUpdateSerial_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: file_path
  {
    const rosidl_runtime_c__String * str = &ros_message->file_path;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _FirmwareUpdateSerial_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _FirmwareUpdateSerial_Request__ros_msg_type * ros_message = static_cast<_FirmwareUpdateSerial_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: file_path
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->file_path.data) {
      rosidl_runtime_c__String__init(&ros_message->file_path);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->file_path,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'file_path'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rokubimini_msgs
size_t get_serialized_size_rokubimini_msgs__srv__FirmwareUpdateSerial_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _FirmwareUpdateSerial_Request__ros_msg_type * ros_message = static_cast<const _FirmwareUpdateSerial_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name file_path
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->file_path.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _FirmwareUpdateSerial_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rokubimini_msgs__srv__FirmwareUpdateSerial_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rokubimini_msgs
size_t max_serialized_size_rokubimini_msgs__srv__FirmwareUpdateSerial_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: file_path
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _FirmwareUpdateSerial_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_rokubimini_msgs__srv__FirmwareUpdateSerial_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_FirmwareUpdateSerial_Request = {
  "rokubimini_msgs::srv",
  "FirmwareUpdateSerial_Request",
  _FirmwareUpdateSerial_Request__cdr_serialize,
  _FirmwareUpdateSerial_Request__cdr_deserialize,
  _FirmwareUpdateSerial_Request__get_serialized_size,
  _FirmwareUpdateSerial_Request__max_serialized_size
};

static rosidl_message_type_support_t _FirmwareUpdateSerial_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_FirmwareUpdateSerial_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rokubimini_msgs, srv, FirmwareUpdateSerial_Request)() {
  return &_FirmwareUpdateSerial_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "rokubimini_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "rokubimini_msgs/srv/detail/firmware_update_serial__struct.h"
// already included above
// #include "rokubimini_msgs/srv/detail/firmware_update_serial__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _FirmwareUpdateSerial_Response__ros_msg_type = rokubimini_msgs__srv__FirmwareUpdateSerial_Response;

static bool _FirmwareUpdateSerial_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _FirmwareUpdateSerial_Response__ros_msg_type * ros_message = static_cast<const _FirmwareUpdateSerial_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: result
  {
    cdr << (ros_message->result ? true : false);
  }

  return true;
}

static bool _FirmwareUpdateSerial_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _FirmwareUpdateSerial_Response__ros_msg_type * ros_message = static_cast<_FirmwareUpdateSerial_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: result
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->result = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rokubimini_msgs
size_t get_serialized_size_rokubimini_msgs__srv__FirmwareUpdateSerial_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _FirmwareUpdateSerial_Response__ros_msg_type * ros_message = static_cast<const _FirmwareUpdateSerial_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name result
  {
    size_t item_size = sizeof(ros_message->result);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _FirmwareUpdateSerial_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rokubimini_msgs__srv__FirmwareUpdateSerial_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rokubimini_msgs
size_t max_serialized_size_rokubimini_msgs__srv__FirmwareUpdateSerial_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: result
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _FirmwareUpdateSerial_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_rokubimini_msgs__srv__FirmwareUpdateSerial_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_FirmwareUpdateSerial_Response = {
  "rokubimini_msgs::srv",
  "FirmwareUpdateSerial_Response",
  _FirmwareUpdateSerial_Response__cdr_serialize,
  _FirmwareUpdateSerial_Response__cdr_deserialize,
  _FirmwareUpdateSerial_Response__get_serialized_size,
  _FirmwareUpdateSerial_Response__max_serialized_size
};

static rosidl_message_type_support_t _FirmwareUpdateSerial_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_FirmwareUpdateSerial_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rokubimini_msgs, srv, FirmwareUpdateSerial_Response)() {
  return &_FirmwareUpdateSerial_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rokubimini_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rokubimini_msgs/srv/firmware_update_serial.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t FirmwareUpdateSerial__callbacks = {
  "rokubimini_msgs::srv",
  "FirmwareUpdateSerial",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rokubimini_msgs, srv, FirmwareUpdateSerial_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rokubimini_msgs, srv, FirmwareUpdateSerial_Response)(),
};

static rosidl_service_type_support_t FirmwareUpdateSerial__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &FirmwareUpdateSerial__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rokubimini_msgs, srv, FirmwareUpdateSerial)() {
  return &FirmwareUpdateSerial__handle;
}

#if defined(__cplusplus)
}
#endif
