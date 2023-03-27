// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mocap_optitrack:msg/PointArrayStamped.idl
// generated code does not contain a copyright notice
#include "mocap_optitrack/msg/detail/point_array_stamped__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `points`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
mocap_optitrack__msg__PointArrayStamped__init(mocap_optitrack__msg__PointArrayStamped * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mocap_optitrack__msg__PointArrayStamped__fini(msg);
    return false;
  }
  // points
  if (!geometry_msgs__msg__Point__Sequence__init(&msg->points, 0)) {
    mocap_optitrack__msg__PointArrayStamped__fini(msg);
    return false;
  }
  return true;
}

void
mocap_optitrack__msg__PointArrayStamped__fini(mocap_optitrack__msg__PointArrayStamped * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // points
  geometry_msgs__msg__Point__Sequence__fini(&msg->points);
}

bool
mocap_optitrack__msg__PointArrayStamped__are_equal(const mocap_optitrack__msg__PointArrayStamped * lhs, const mocap_optitrack__msg__PointArrayStamped * rhs)
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
  // points
  if (!geometry_msgs__msg__Point__Sequence__are_equal(
      &(lhs->points), &(rhs->points)))
  {
    return false;
  }
  return true;
}

bool
mocap_optitrack__msg__PointArrayStamped__copy(
  const mocap_optitrack__msg__PointArrayStamped * input,
  mocap_optitrack__msg__PointArrayStamped * output)
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
  // points
  if (!geometry_msgs__msg__Point__Sequence__copy(
      &(input->points), &(output->points)))
  {
    return false;
  }
  return true;
}

mocap_optitrack__msg__PointArrayStamped *
mocap_optitrack__msg__PointArrayStamped__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mocap_optitrack__msg__PointArrayStamped * msg = (mocap_optitrack__msg__PointArrayStamped *)allocator.allocate(sizeof(mocap_optitrack__msg__PointArrayStamped), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mocap_optitrack__msg__PointArrayStamped));
  bool success = mocap_optitrack__msg__PointArrayStamped__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mocap_optitrack__msg__PointArrayStamped__destroy(mocap_optitrack__msg__PointArrayStamped * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mocap_optitrack__msg__PointArrayStamped__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mocap_optitrack__msg__PointArrayStamped__Sequence__init(mocap_optitrack__msg__PointArrayStamped__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mocap_optitrack__msg__PointArrayStamped * data = NULL;

  if (size) {
    data = (mocap_optitrack__msg__PointArrayStamped *)allocator.zero_allocate(size, sizeof(mocap_optitrack__msg__PointArrayStamped), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mocap_optitrack__msg__PointArrayStamped__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mocap_optitrack__msg__PointArrayStamped__fini(&data[i - 1]);
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
mocap_optitrack__msg__PointArrayStamped__Sequence__fini(mocap_optitrack__msg__PointArrayStamped__Sequence * array)
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
      mocap_optitrack__msg__PointArrayStamped__fini(&array->data[i]);
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

mocap_optitrack__msg__PointArrayStamped__Sequence *
mocap_optitrack__msg__PointArrayStamped__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mocap_optitrack__msg__PointArrayStamped__Sequence * array = (mocap_optitrack__msg__PointArrayStamped__Sequence *)allocator.allocate(sizeof(mocap_optitrack__msg__PointArrayStamped__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mocap_optitrack__msg__PointArrayStamped__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mocap_optitrack__msg__PointArrayStamped__Sequence__destroy(mocap_optitrack__msg__PointArrayStamped__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mocap_optitrack__msg__PointArrayStamped__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mocap_optitrack__msg__PointArrayStamped__Sequence__are_equal(const mocap_optitrack__msg__PointArrayStamped__Sequence * lhs, const mocap_optitrack__msg__PointArrayStamped__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mocap_optitrack__msg__PointArrayStamped__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mocap_optitrack__msg__PointArrayStamped__Sequence__copy(
  const mocap_optitrack__msg__PointArrayStamped__Sequence * input,
  mocap_optitrack__msg__PointArrayStamped__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mocap_optitrack__msg__PointArrayStamped);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mocap_optitrack__msg__PointArrayStamped * data =
      (mocap_optitrack__msg__PointArrayStamped *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mocap_optitrack__msg__PointArrayStamped__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mocap_optitrack__msg__PointArrayStamped__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mocap_optitrack__msg__PointArrayStamped__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
