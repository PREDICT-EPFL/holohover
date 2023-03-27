// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from rokubimini_msgs:srv/FirmwareUpdateSerial.idl
// generated code does not contain a copyright notice
#include "rokubimini_msgs/srv/detail/firmware_update_serial__rosidl_typesupport_fastrtps_cpp.hpp"
#include "rokubimini_msgs/srv/detail/firmware_update_serial__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace rokubimini_msgs
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rokubimini_msgs
cdr_serialize(
  const rokubimini_msgs::srv::FirmwareUpdateSerial_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: file_path
  cdr << ros_message.file_path;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rokubimini_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  rokubimini_msgs::srv::FirmwareUpdateSerial_Request & ros_message)
{
  // Member: file_path
  cdr >> ros_message.file_path;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rokubimini_msgs
get_serialized_size(
  const rokubimini_msgs::srv::FirmwareUpdateSerial_Request & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: file_path
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.file_path.size() + 1);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rokubimini_msgs
max_serialized_size_FirmwareUpdateSerial_Request(
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


  // Member: file_path
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

static bool _FirmwareUpdateSerial_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const rokubimini_msgs::srv::FirmwareUpdateSerial_Request *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _FirmwareUpdateSerial_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<rokubimini_msgs::srv::FirmwareUpdateSerial_Request *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _FirmwareUpdateSerial_Request__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const rokubimini_msgs::srv::FirmwareUpdateSerial_Request *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _FirmwareUpdateSerial_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_FirmwareUpdateSerial_Request(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _FirmwareUpdateSerial_Request__callbacks = {
  "rokubimini_msgs::srv",
  "FirmwareUpdateSerial_Request",
  _FirmwareUpdateSerial_Request__cdr_serialize,
  _FirmwareUpdateSerial_Request__cdr_deserialize,
  _FirmwareUpdateSerial_Request__get_serialized_size,
  _FirmwareUpdateSerial_Request__max_serialized_size
};

static rosidl_message_type_support_t _FirmwareUpdateSerial_Request__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_FirmwareUpdateSerial_Request__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace rokubimini_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_rokubimini_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<rokubimini_msgs::srv::FirmwareUpdateSerial_Request>()
{
  return &rokubimini_msgs::srv::typesupport_fastrtps_cpp::_FirmwareUpdateSerial_Request__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rokubimini_msgs, srv, FirmwareUpdateSerial_Request)() {
  return &rokubimini_msgs::srv::typesupport_fastrtps_cpp::_FirmwareUpdateSerial_Request__handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include <limits>
// already included above
// #include <stdexcept>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
// already included above
// #include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace rokubimini_msgs
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rokubimini_msgs
cdr_serialize(
  const rokubimini_msgs::srv::FirmwareUpdateSerial_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: result
  cdr << (ros_message.result ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rokubimini_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  rokubimini_msgs::srv::FirmwareUpdateSerial_Response & ros_message)
{
  // Member: result
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.result = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rokubimini_msgs
get_serialized_size(
  const rokubimini_msgs::srv::FirmwareUpdateSerial_Response & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: result
  {
    size_t item_size = sizeof(ros_message.result);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rokubimini_msgs
max_serialized_size_FirmwareUpdateSerial_Response(
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


  // Member: result
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static bool _FirmwareUpdateSerial_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const rokubimini_msgs::srv::FirmwareUpdateSerial_Response *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _FirmwareUpdateSerial_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<rokubimini_msgs::srv::FirmwareUpdateSerial_Response *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _FirmwareUpdateSerial_Response__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const rokubimini_msgs::srv::FirmwareUpdateSerial_Response *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _FirmwareUpdateSerial_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_FirmwareUpdateSerial_Response(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _FirmwareUpdateSerial_Response__callbacks = {
  "rokubimini_msgs::srv",
  "FirmwareUpdateSerial_Response",
  _FirmwareUpdateSerial_Response__cdr_serialize,
  _FirmwareUpdateSerial_Response__cdr_deserialize,
  _FirmwareUpdateSerial_Response__get_serialized_size,
  _FirmwareUpdateSerial_Response__max_serialized_size
};

static rosidl_message_type_support_t _FirmwareUpdateSerial_Response__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_FirmwareUpdateSerial_Response__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace rokubimini_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_rokubimini_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<rokubimini_msgs::srv::FirmwareUpdateSerial_Response>()
{
  return &rokubimini_msgs::srv::typesupport_fastrtps_cpp::_FirmwareUpdateSerial_Response__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rokubimini_msgs, srv, FirmwareUpdateSerial_Response)() {
  return &rokubimini_msgs::srv::typesupport_fastrtps_cpp::_FirmwareUpdateSerial_Response__handle;
}

#ifdef __cplusplus
}
#endif

#include "rmw/error_handling.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support_decl.hpp"

namespace rokubimini_msgs
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

static service_type_support_callbacks_t _FirmwareUpdateSerial__callbacks = {
  "rokubimini_msgs::srv",
  "FirmwareUpdateSerial",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rokubimini_msgs, srv, FirmwareUpdateSerial_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rokubimini_msgs, srv, FirmwareUpdateSerial_Response)(),
};

static rosidl_service_type_support_t _FirmwareUpdateSerial__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_FirmwareUpdateSerial__callbacks,
  get_service_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace rokubimini_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_rokubimini_msgs
const rosidl_service_type_support_t *
get_service_type_support_handle<rokubimini_msgs::srv::FirmwareUpdateSerial>()
{
  return &rokubimini_msgs::srv::typesupport_fastrtps_cpp::_FirmwareUpdateSerial__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rokubimini_msgs, srv, FirmwareUpdateSerial)() {
  return &rokubimini_msgs::srv::typesupport_fastrtps_cpp::_FirmwareUpdateSerial__handle;
}

#ifdef __cplusplus
}
#endif
