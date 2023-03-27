// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from holohover_msgs:msg/Attitude.idl
// generated code does not contain a copyright notice
#include "holohover_msgs/msg/detail/attitude__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
holohover_msgs__msg__Attitude__init(holohover_msgs__msg__Attitude * msg)
{
  if (!msg) {
    return false;
  }
  // roll
  // pitch
  // yaw
  return true;
}

void
holohover_msgs__msg__Attitude__fini(holohover_msgs__msg__Attitude * msg)
{
  if (!msg) {
    return;
  }
  // roll
  // pitch
  // yaw
}

bool
holohover_msgs__msg__Attitude__are_equal(const holohover_msgs__msg__Attitude * lhs, const holohover_msgs__msg__Attitude * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // roll
  if (lhs->roll != rhs->roll) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  return true;
}

bool
holohover_msgs__msg__Attitude__copy(
  const holohover_msgs__msg__Attitude * input,
  holohover_msgs__msg__Attitude * output)
{
  if (!input || !output) {
    return false;
  }
  // roll
  output->roll = input->roll;
  // pitch
  output->pitch = input->pitch;
  // yaw
  output->yaw = input->yaw;
  return true;
}

holohover_msgs__msg__Attitude *
holohover_msgs__msg__Attitude__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  holohover_msgs__msg__Attitude * msg = (holohover_msgs__msg__Attitude *)allocator.allocate(sizeof(holohover_msgs__msg__Attitude), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(holohover_msgs__msg__Attitude));
  bool success = holohover_msgs__msg__Attitude__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
holohover_msgs__msg__Attitude__destroy(holohover_msgs__msg__Attitude * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    holohover_msgs__msg__Attitude__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
holohover_msgs__msg__Attitude__Sequence__init(holohover_msgs__msg__Attitude__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  holohover_msgs__msg__Attitude * data = NULL;

  if (size) {
    data = (holohover_msgs__msg__Attitude *)allocator.zero_allocate(size, sizeof(holohover_msgs__msg__Attitude), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = holohover_msgs__msg__Attitude__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        holohover_msgs__msg__Attitude__fini(&data[i - 1]);
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
holohover_msgs__msg__Attitude__Sequence__fini(holohover_msgs__msg__Attitude__Sequence * array)
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
      holohover_msgs__msg__Attitude__fini(&array->data[i]);
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

holohover_msgs__msg__Attitude__Sequence *
holohover_msgs__msg__Attitude__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  holohover_msgs__msg__Attitude__Sequence * array = (holohover_msgs__msg__Attitude__Sequence *)allocator.allocate(sizeof(holohover_msgs__msg__Attitude__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = holohover_msgs__msg__Attitude__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
holohover_msgs__msg__Attitude__Sequence__destroy(holohover_msgs__msg__Attitude__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    holohover_msgs__msg__Attitude__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
holohover_msgs__msg__Attitude__Sequence__are_equal(const holohover_msgs__msg__Attitude__Sequence * lhs, const holohover_msgs__msg__Attitude__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!holohover_msgs__msg__Attitude__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
holohover_msgs__msg__Attitude__Sequence__copy(
  const holohover_msgs__msg__Attitude__Sequence * input,
  holohover_msgs__msg__Attitude__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(holohover_msgs__msg__Attitude);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    holohover_msgs__msg__Attitude * data =
      (holohover_msgs__msg__Attitude *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!holohover_msgs__msg__Attitude__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          holohover_msgs__msg__Attitude__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!holohover_msgs__msg__Attitude__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
