// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rokubimini_msgs:msg/Reading.idl
// generated code does not contain a copyright notice

#ifndef ROKUBIMINI_MSGS__MSG__DETAIL__READING__STRUCT_H_
#define ROKUBIMINI_MSGS__MSG__DETAIL__READING__STRUCT_H_

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
// Member 'imu'
// Member 'external_imu'
#include "sensor_msgs/msg/detail/imu__struct.h"
// Member 'wrench'
#include "geometry_msgs/msg/detail/wrench_stamped__struct.h"
// Member 'temperature'
#include "sensor_msgs/msg/detail/temperature__struct.h"

/// Struct defined in msg/Reading in the package rokubimini_msgs.
/**
  * Reading
 */
typedef struct rokubimini_msgs__msg__Reading
{
  /// Message header.
  std_msgs__msg__Header header;
  /// Statusword.
  uint32_t statusword;
  sensor_msgs__msg__Imu imu;
  geometry_msgs__msg__WrenchStamped wrench;
  sensor_msgs__msg__Imu external_imu;
  bool is_force_torque_saturated;
  sensor_msgs__msg__Temperature temperature;
} rokubimini_msgs__msg__Reading;

// Struct for a sequence of rokubimini_msgs__msg__Reading.
typedef struct rokubimini_msgs__msg__Reading__Sequence
{
  rokubimini_msgs__msg__Reading * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rokubimini_msgs__msg__Reading__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROKUBIMINI_MSGS__MSG__DETAIL__READING__STRUCT_H_
