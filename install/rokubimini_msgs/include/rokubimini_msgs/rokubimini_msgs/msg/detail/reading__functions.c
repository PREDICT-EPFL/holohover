// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rokubimini_msgs:msg/Reading.idl
// generated code does not contain a copyright notice
#include "rokubimini_msgs/msg/detail/reading__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `imu`
// Member `external_imu`
#include "sensor_msgs/msg/detail/imu__functions.h"
// Member `wrench`
#include "geometry_msgs/msg/detail/wrench_stamped__functions.h"
// Member `temperature`
#include "sensor_msgs/msg/detail/temperature__functions.h"

bool
rokubimini_msgs__msg__Reading__init(rokubimini_msgs__msg__Reading * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    rokubimini_msgs__msg__Reading__fini(msg);
    return false;
  }
  // statusword
  // imu
  if (!sensor_msgs__msg__Imu__init(&msg->imu)) {
    rokubimini_msgs__msg__Reading__fini(msg);
    return false;
  }
  // wrench
  if (!geometry_msgs__msg__WrenchStamped__init(&msg->wrench)) {
    rokubimini_msgs__msg__Reading__fini(msg);
    return false;
  }
  // external_imu
  if (!sensor_msgs__msg__Imu__init(&msg->external_imu)) {
    rokubimini_msgs__msg__Reading__fini(msg);
    return false;
  }
  // is_force_torque_saturated
  // temperature
  if (!sensor_msgs__msg__Temperature__init(&msg->temperature)) {
    rokubimini_msgs__msg__Reading__fini(msg);
    return false;
  }
  return true;
}

void
rokubimini_msgs__msg__Reading__fini(rokubimini_msgs__msg__Reading * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // statusword
  // imu
  sensor_msgs__msg__Imu__fini(&msg->imu);
  // wrench
  geometry_msgs__msg__WrenchStamped__fini(&msg->wrench);
  // external_imu
  sensor_msgs__msg__Imu__fini(&msg->external_imu);
  // is_force_torque_saturated
  // temperature
  sensor_msgs__msg__Temperature__fini(&msg->temperature);
}

bool
rokubimini_msgs__msg__Reading__are_equal(const rokubimini_msgs__msg__Reading * lhs, const rokubimini_msgs__msg__Reading * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // statusword
  if (lhs->statusword != rhs->statusword) {
    return false;
  }
  // imu
  if (!sensor_msgs__msg__Imu__are_equal(
      &(lhs->imu), &(rhs->imu)))
  {
    return false;
  }
  // wrench
  if (!geometry_msgs__msg__WrenchStamped__are_equal(
      &(lhs->wrench), &(rhs->wrench)))
  {
    return false;
  }
  // external_imu
  if (!sensor_msgs__msg__Imu__are_equal(
      &(lhs->external_imu), &(rhs->external_imu)))
  {
    return false;
  }
  // is_force_torque_saturated
  if (lhs->is_force_torque_saturated != rhs->is_force_torque_saturated) {
    return false;
  }
  // temperature
  if (!sensor_msgs__msg__Temperature__are_equal(
      &(lhs->temperature), &(rhs->temperature)))
  {
    return false;
  }
  return true;
}

bool
rokubimini_msgs__msg__Reading__copy(
  const rokubimini_msgs__msg__Reading * input,
  rokubimini_msgs__msg__Reading * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // statusword
  output->statusword = input->statusword;
  // imu
  if (!sensor_msgs__msg__Imu__copy(
      &(input->imu), &(output->imu)))
  {
    return false;
  }
  // wrench
  if (!geometry_msgs__msg__WrenchStamped__copy(
      &(input->wrench), &(output->wrench)))
  {
    return false;
  }
  // external_imu
  if (!sensor_msgs__msg__Imu__copy(
      &(input->external_imu), &(output->external_imu)))
  {
    return false;
  }
  // is_force_torque_saturated
  output->is_force_torque_saturated = input->is_force_torque_saturated;
  // temperature
  if (!sensor_msgs__msg__Temperature__copy(
      &(input->temperature), &(output->temperature)))
  {
    return false;
  }
  return true;
}

rokubimini_msgs__msg__Reading *
rokubimini_msgs__msg__Reading__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rokubimini_msgs__msg__Reading * msg = (rokubimini_msgs__msg__Reading *)allocator.allocate(sizeof(rokubimini_msgs__msg__Reading), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rokubimini_msgs__msg__Reading));
  bool success = rokubimini_msgs__msg__Reading__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rokubimini_msgs__msg__Reading__destroy(rokubimini_msgs__msg__Reading * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rokubimini_msgs__msg__Reading__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rokubimini_msgs__msg__Reading__Sequence__init(rokubimini_msgs__msg__Reading__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rokubimini_msgs__msg__Reading * data = NULL;

  if (size) {
    data = (rokubimini_msgs__msg__Reading *)allocator.zero_allocate(size, sizeof(rokubimini_msgs__msg__Reading), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rokubimini_msgs__msg__Reading__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rokubimini_msgs__msg__Reading__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rokubimini_msgs__msg__Reading__Sequence__fini(rokubimini_msgs__msg__Reading__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rokubimini_msgs__msg__Reading__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rokubimini_msgs__msg__Reading__Sequence *
rokubimini_msgs__msg__Reading__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rokubimini_msgs__msg__Reading__Sequence * array = (rokubimini_msgs__msg__Reading__Sequence *)allocator.allocate(sizeof(rokubimini_msgs__msg__Reading__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rokubimini_msgs__msg__Reading__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rokubimini_msgs__msg__Reading__Sequence__destroy(rokubimini_msgs__msg__Reading__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rokubimini_msgs__msg__Reading__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rokubimini_msgs__msg__Reading__Sequence__are_equal(const rokubimini_msgs__msg__Reading__Sequence * lhs, const rokubimini_msgs__msg__Reading__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rokubimini_msgs__msg__Reading__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rokubimini_msgs__msg__Reading__Sequence__copy(
  const rokubimini_msgs__msg__Reading__Sequence * input,
  rokubimini_msgs__msg__Reading__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rokubimini_msgs__msg__Reading);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rokubimini_msgs__msg__Reading * data =
      (rokubimini_msgs__msg__Reading *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rokubimini_msgs__msg__Reading__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rokubimini_msgs__msg__Reading__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rokubimini_msgs__msg__Reading__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
