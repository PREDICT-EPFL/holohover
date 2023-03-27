// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rokubimini_msgs:srv/FirmwareUpdateEthercat.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rokubimini_msgs/srv/detail/firmware_update_ethercat__rosidl_typesupport_introspection_c.h"
#include "rokubimini_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rokubimini_msgs/srv/detail/firmware_update_ethercat__functions.h"
#include "rokubimini_msgs/srv/detail/firmware_update_ethercat__struct.h"


// Include directives for member types
// Member `file_name`
// Member `file_path`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__init(message_memory);
}

void rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_Request_fini_function(void * message_memory)
{
  rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_Request_message_member_array[3] = {
  {
    "file_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rokubimini_msgs__srv__FirmwareUpdateEthercat_Request, file_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "file_path",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rokubimini_msgs__srv__FirmwareUpdateEthercat_Request, file_path),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "password",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rokubimini_msgs__srv__FirmwareUpdateEthercat_Request, password),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_Request_message_members = {
  "rokubimini_msgs__srv",  // message namespace
  "FirmwareUpdateEthercat_Request",  // message name
  3,  // number of fields
  sizeof(rokubimini_msgs__srv__FirmwareUpdateEthercat_Request),
  rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_Request_message_member_array,  // message members
  rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_Request_message_type_support_handle = {
  0,
  &rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rokubimini_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rokubimini_msgs, srv, FirmwareUpdateEthercat_Request)() {
  if (!rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_Request_message_type_support_handle.typesupport_identifier) {
    rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rokubimini_msgs/srv/detail/firmware_update_ethercat__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rokubimini_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rokubimini_msgs/srv/detail/firmware_update_ethercat__functions.h"
// already included above
// #include "rokubimini_msgs/srv/detail/firmware_update_ethercat__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__init(message_memory);
}

void rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_Response_fini_function(void * message_memory)
{
  rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_Response_message_member_array[1] = {
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rokubimini_msgs__srv__FirmwareUpdateEthercat_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_Response_message_members = {
  "rokubimini_msgs__srv",  // message namespace
  "FirmwareUpdateEthercat_Response",  // message name
  1,  // number of fields
  sizeof(rokubimini_msgs__srv__FirmwareUpdateEthercat_Response),
  rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_Response_message_member_array,  // message members
  rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_Response_message_type_support_handle = {
  0,
  &rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rokubimini_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rokubimini_msgs, srv, FirmwareUpdateEthercat_Response)() {
  if (!rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_Response_message_type_support_handle.typesupport_identifier) {
    rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rokubimini_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rokubimini_msgs/srv/detail/firmware_update_ethercat__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers rokubimini_msgs__srv__detail__firmware_update_ethercat__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_service_members = {
  "rokubimini_msgs__srv",  // service namespace
  "FirmwareUpdateEthercat",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // rokubimini_msgs__srv__detail__firmware_update_ethercat__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_Request_message_type_support_handle,
  NULL  // response message
  // rokubimini_msgs__srv__detail__firmware_update_ethercat__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_Response_message_type_support_handle
};

static rosidl_service_type_support_t rokubimini_msgs__srv__detail__firmware_update_ethercat__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_service_type_support_handle = {
  0,
  &rokubimini_msgs__srv__detail__firmware_update_ethercat__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rokubimini_msgs, srv, FirmwareUpdateEthercat_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rokubimini_msgs, srv, FirmwareUpdateEthercat_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rokubimini_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rokubimini_msgs, srv, FirmwareUpdateEthercat)() {
  if (!rokubimini_msgs__srv__detail__firmware_update_ethercat__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_service_type_support_handle.typesupport_identifier) {
    rokubimini_msgs__srv__detail__firmware_update_ethercat__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)rokubimini_msgs__srv__detail__firmware_update_ethercat__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rokubimini_msgs, srv, FirmwareUpdateEthercat_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rokubimini_msgs, srv, FirmwareUpdateEthercat_Response)()->data;
  }

  return &rokubimini_msgs__srv__detail__firmware_update_ethercat__rosidl_typesupport_introspection_c__FirmwareUpdateEthercat_service_type_support_handle;
}
