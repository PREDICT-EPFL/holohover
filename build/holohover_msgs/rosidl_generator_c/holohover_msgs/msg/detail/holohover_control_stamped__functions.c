// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from holohover_msgs:msg/HolohoverControlStamped.idl
// generated code does not contain a copyright notice
#include "holohover_msgs/msg/detail/holohover_control_stamped__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
holohover_msgs__msg__HolohoverControlStamped__init(holohover_msgs__msg__HolohoverControlStamped * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    holohover_msgs__msg__HolohoverControlStamped__fini(msg);
    return false;
  }
  // motor_a_1
  // motor_a_2
  // motor_b_1
  // motor_b_2
  // motor_c_1
  // motor_c_2
  return true;
}

void
holohover_msgs__msg__HolohoverControlStamped__fini(holohover_msgs__msg__HolohoverControlStamped * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // motor_a_1
  // motor_a_2
  // motor_b_1
  // motor_b_2
  // motor_c_1
  // motor_c_2
}

bool
holohover_msgs__msg__HolohoverControlStamped__are_equal(const holohover_msgs__msg__HolohoverControlStamped * lhs, const holohover_msgs__msg__HolohoverControlStamped * rhs)
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
  // motor_a_1
  if (lhs->motor_a_1 != rhs->motor_a_1) {
    return false;
  }
  // motor_a_2
  if (lhs->motor_a_2 != rhs->motor_a_2) {
    return false;
  }
  // motor_b_1
  if (lhs->motor_b_1 != rhs->motor_b_1) {
    return false;
  }
  // motor_b_2
  if (lhs->motor_b_2 != rhs->motor_b_2) {
    return false;
  }
  // motor_c_1
  if (lhs->motor_c_1 != rhs->motor_c_1) {
    return false;
  }
  // motor_c_2
  if (lhs->motor_c_2 != rhs->motor_c_2) {
    return false;
  }
  return true;
}

bool
holohover_msgs__msg__HolohoverControlStamped__copy(
  const holohover_msgs__msg__HolohoverControlStamped * input,
  holohover_msgs__msg__HolohoverControlStamped * output)
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
  // motor_a_1
  output->motor_a_1 = input->motor_a_1;
  // motor_a_2
  output->motor_a_2 = input->motor_a_2;
  // motor_b_1
  output->motor_b_1 = input->motor_b_1;
  // motor_b_2
  output->motor_b_2 = input->motor_b_2;
  // motor_c_1
  output->motor_c_1 = input->motor_c_1;
  // motor_c_2
  output->motor_c_2 = input->motor_c_2;
  return true;
}

holohover_msgs__msg__HolohoverControlStamped *
holohover_msgs__msg__HolohoverControlStamped__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  holohover_msgs__msg__HolohoverControlStamped * msg = (holohover_msgs__msg__HolohoverControlStamped *)allocator.allocate(sizeof(holohover_msgs__msg__HolohoverControlStamped), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(holohover_msgs__msg__HolohoverControlStamped));
  bool success = holohover_msgs__msg__HolohoverControlStamped__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
holohover_msgs__msg__HolohoverControlStamped__destroy(holohover_msgs__msg__HolohoverControlStamped * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    holohover_msgs__msg__HolohoverControlStamped__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
holohover_msgs__msg__HolohoverControlStamped__Sequence__init(holohover_msgs__msg__HolohoverControlStamped__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  holohover_msgs__msg__HolohoverControlStamped * data = NULL;

  if (size) {
    data = (holohover_msgs__msg__HolohoverControlStamped *)allocator.zero_allocate(size, sizeof(holohover_msgs__msg__HolohoverControlStamped), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = holohover_msgs__msg__HolohoverControlStamped__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        holohover_msgs__msg__HolohoverControlStamped__fini(&data[i - 1]);
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
holohover_msgs__msg__HolohoverControlStamped__Sequence__fini(holohover_msgs__msg__HolohoverControlStamped__Sequence * array)
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
      holohover_msgs__msg__HolohoverControlStamped__fini(&array->data[i]);
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

holohover_msgs__msg__HolohoverControlStamped__Sequence *
holohover_msgs__msg__HolohoverControlStamped__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  holohover_msgs__msg__HolohoverControlStamped__Sequence * array = (holohover_msgs__msg__HolohoverControlStamped__Sequence *)allocator.allocate(sizeof(holohover_msgs__msg__HolohoverControlStamped__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = holohover_msgs__msg__HolohoverControlStamped__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
holohover_msgs__msg__HolohoverControlStamped__Sequence__destroy(holohover_msgs__msg__HolohoverControlStamped__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    holohover_msgs__msg__HolohoverControlStamped__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
holohover_msgs__msg__HolohoverControlStamped__Sequence__are_equal(const holohover_msgs__msg__HolohoverControlStamped__Sequence * lhs, const holohover_msgs__msg__HolohoverControlStamped__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!holohover_msgs__msg__HolohoverControlStamped__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
holohover_msgs__msg__HolohoverControlStamped__Sequence__copy(
  const holohover_msgs__msg__HolohoverControlStamped__Sequence * input,
  holohover_msgs__msg__HolohoverControlStamped__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(holohover_msgs__msg__HolohoverControlStamped);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    holohover_msgs__msg__HolohoverControlStamped * data =
      (holohover_msgs__msg__HolohoverControlStamped *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!holohover_msgs__msg__HolohoverControlStamped__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          holohover_msgs__msg__HolohoverControlStamped__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!holohover_msgs__msg__HolohoverControlStamped__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
