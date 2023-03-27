// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from holohover_msgs:msg/HolohoverIMUStamped.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "holohover_msgs/msg/detail/holohover_imu_stamped__rosidl_typesupport_introspection_c.h"
#include "holohover_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "holohover_msgs/msg/detail/holohover_imu_stamped__functions.h"
#include "holohover_msgs/msg/detail/holohover_imu_stamped__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `atti`
#include "holohover_msgs/msg/attitude.h"
// Member `atti`
#include "holohover_msgs/msg/detail/attitude__rosidl_typesupport_introspection_c.h"
// Member `acc`
// Member `gyro`
#include "geometry_msgs/msg/vector3.h"
// Member `acc`
// Member `gyro`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void holohover_msgs__msg__HolohoverIMUStamped__rosidl_typesupport_introspection_c__HolohoverIMUStamped_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  holohover_msgs__msg__HolohoverIMUStamped__init(message_memory);
}

void holohover_msgs__msg__HolohoverIMUStamped__rosidl_typesupport_introspection_c__HolohoverIMUStamped_fini_function(void * message_memory)
{
  holohover_msgs__msg__HolohoverIMUStamped__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember holohover_msgs__msg__HolohoverIMUStamped__rosidl_typesupport_introspection_c__HolohoverIMUStamped_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(holohover_msgs__msg__HolohoverIMUStamped, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "atti",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(holohover_msgs__msg__HolohoverIMUStamped, atti),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "acc",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(holohover_msgs__msg__HolohoverIMUStamped, acc),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gyro",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(holohover_msgs__msg__HolohoverIMUStamped, gyro),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers holohover_msgs__msg__HolohoverIMUStamped__rosidl_typesupport_introspection_c__HolohoverIMUStamped_message_members = {
  "holohover_msgs__msg",  // message namespace
  "HolohoverIMUStamped",  // message name
  4,  // number of fields
  sizeof(holohover_msgs__msg__HolohoverIMUStamped),
  holohover_msgs__msg__HolohoverIMUStamped__rosidl_typesupport_introspection_c__HolohoverIMUStamped_message_member_array,  // message members
  holohover_msgs__msg__HolohoverIMUStamped__rosidl_typesupport_introspection_c__HolohoverIMUStamped_init_function,  // function to initialize message memory (memory has to be allocated)
  holohover_msgs__msg__HolohoverIMUStamped__rosidl_typesupport_introspection_c__HolohoverIMUStamped_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t holohover_msgs__msg__HolohoverIMUStamped__rosidl_typesupport_introspection_c__HolohoverIMUStamped_message_type_support_handle = {
  0,
  &holohover_msgs__msg__HolohoverIMUStamped__rosidl_typesupport_introspection_c__HolohoverIMUStamped_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_holohover_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, holohover_msgs, msg, HolohoverIMUStamped)() {
  holohover_msgs__msg__HolohoverIMUStamped__rosidl_typesupport_introspection_c__HolohoverIMUStamped_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  holohover_msgs__msg__HolohoverIMUStamped__rosidl_typesupport_introspection_c__HolohoverIMUStamped_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, holohover_msgs, msg, Attitude)();
  holohover_msgs__msg__HolohoverIMUStamped__rosidl_typesupport_introspection_c__HolohoverIMUStamped_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  holohover_msgs__msg__HolohoverIMUStamped__rosidl_typesupport_introspection_c__HolohoverIMUStamped_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  if (!holohover_msgs__msg__HolohoverIMUStamped__rosidl_typesupport_introspection_c__HolohoverIMUStamped_message_type_support_handle.typesupport_identifier) {
    holohover_msgs__msg__HolohoverIMUStamped__rosidl_typesupport_introspection_c__HolohoverIMUStamped_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &holohover_msgs__msg__HolohoverIMUStamped__rosidl_typesupport_introspection_c__HolohoverIMUStamped_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
