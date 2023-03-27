// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rokubimini_msgs:msg/Reading.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rokubimini_msgs/msg/detail/reading__rosidl_typesupport_introspection_c.h"
#include "rokubimini_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rokubimini_msgs/msg/detail/reading__functions.h"
#include "rokubimini_msgs/msg/detail/reading__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `imu`
// Member `external_imu`
#include "sensor_msgs/msg/imu.h"
// Member `imu`
// Member `external_imu`
#include "sensor_msgs/msg/detail/imu__rosidl_typesupport_introspection_c.h"
// Member `wrench`
#include "geometry_msgs/msg/wrench_stamped.h"
// Member `wrench`
#include "geometry_msgs/msg/detail/wrench_stamped__rosidl_typesupport_introspection_c.h"
// Member `temperature`
#include "sensor_msgs/msg/temperature.h"
// Member `temperature`
#include "sensor_msgs/msg/detail/temperature__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rokubimini_msgs__msg__Reading__rosidl_typesupport_introspection_c__Reading_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rokubimini_msgs__msg__Reading__init(message_memory);
}

void rokubimini_msgs__msg__Reading__rosidl_typesupport_introspection_c__Reading_fini_function(void * message_memory)
{
  rokubimini_msgs__msg__Reading__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rokubimini_msgs__msg__Reading__rosidl_typesupport_introspection_c__Reading_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rokubimini_msgs__msg__Reading, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "statusword",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rokubimini_msgs__msg__Reading, statusword),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "imu",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rokubimini_msgs__msg__Reading, imu),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "wrench",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rokubimini_msgs__msg__Reading, wrench),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "external_imu",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rokubimini_msgs__msg__Reading, external_imu),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_force_torque_saturated",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rokubimini_msgs__msg__Reading, is_force_torque_saturated),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "temperature",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rokubimini_msgs__msg__Reading, temperature),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rokubimini_msgs__msg__Reading__rosidl_typesupport_introspection_c__Reading_message_members = {
  "rokubimini_msgs__msg",  // message namespace
  "Reading",  // message name
  7,  // number of fields
  sizeof(rokubimini_msgs__msg__Reading),
  rokubimini_msgs__msg__Reading__rosidl_typesupport_introspection_c__Reading_message_member_array,  // message members
  rokubimini_msgs__msg__Reading__rosidl_typesupport_introspection_c__Reading_init_function,  // function to initialize message memory (memory has to be allocated)
  rokubimini_msgs__msg__Reading__rosidl_typesupport_introspection_c__Reading_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rokubimini_msgs__msg__Reading__rosidl_typesupport_introspection_c__Reading_message_type_support_handle = {
  0,
  &rokubimini_msgs__msg__Reading__rosidl_typesupport_introspection_c__Reading_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rokubimini_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rokubimini_msgs, msg, Reading)() {
  rokubimini_msgs__msg__Reading__rosidl_typesupport_introspection_c__Reading_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  rokubimini_msgs__msg__Reading__rosidl_typesupport_introspection_c__Reading_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Imu)();
  rokubimini_msgs__msg__Reading__rosidl_typesupport_introspection_c__Reading_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, WrenchStamped)();
  rokubimini_msgs__msg__Reading__rosidl_typesupport_introspection_c__Reading_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Imu)();
  rokubimini_msgs__msg__Reading__rosidl_typesupport_introspection_c__Reading_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Temperature)();
  if (!rokubimini_msgs__msg__Reading__rosidl_typesupport_introspection_c__Reading_message_type_support_handle.typesupport_identifier) {
    rokubimini_msgs__msg__Reading__rosidl_typesupport_introspection_c__Reading_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rokubimini_msgs__msg__Reading__rosidl_typesupport_introspection_c__Reading_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
