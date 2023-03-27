// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rokubimini_msgs:srv/SetSensorConfiguration.idl
// generated code does not contain a copyright notice
#include "rokubimini_msgs/srv/detail/set_sensor_configuration__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
rokubimini_msgs__srv__SetSensorConfiguration_Request__init(rokubimini_msgs__srv__SetSensorConfiguration_Request * msg)
{
  if (!msg) {
    return false;
  }
  // a
  return true;
}

void
rokubimini_msgs__srv__SetSensorConfiguration_Request__fini(rokubimini_msgs__srv__SetSensorConfiguration_Request * msg)
{
  if (!msg) {
    return;
  }
  // a
}

bool
rokubimini_msgs__srv__SetSensorConfiguration_Request__are_equal(const rokubimini_msgs__srv__SetSensorConfiguration_Request * lhs, const rokubimini_msgs__srv__SetSensorConfiguration_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // a
  if (lhs->a != rhs->a) {
    return false;
  }
  return true;
}

bool
rokubimini_msgs__srv__SetSensorConfiguration_Request__copy(
  const rokubimini_msgs__srv__SetSensorConfiguration_Request * input,
  rokubimini_msgs__srv__SetSensorConfiguration_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // a
  output->a = input->a;
  return true;
}

rokubimini_msgs__srv__SetSensorConfiguration_Request *
rokubimini_msgs__srv__SetSensorConfiguration_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rokubimini_msgs__srv__SetSensorConfiguration_Request * msg = (rokubimini_msgs__srv__SetSensorConfiguration_Request *)allocator.allocate(sizeof(rokubimini_msgs__srv__SetSensorConfiguration_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rokubimini_msgs__srv__SetSensorConfiguration_Request));
  bool success = rokubimini_msgs__srv__SetSensorConfiguration_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rokubimini_msgs__srv__SetSensorConfiguration_Request__destroy(rokubimini_msgs__srv__SetSensorConfiguration_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rokubimini_msgs__srv__SetSensorConfiguration_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rokubimini_msgs__srv__SetSensorConfiguration_Request__Sequence__init(rokubimini_msgs__srv__SetSensorConfiguration_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rokubimini_msgs__srv__SetSensorConfiguration_Request * data = NULL;

  if (size) {
    data = (rokubimini_msgs__srv__SetSensorConfiguration_Request *)allocator.zero_allocate(size, sizeof(rokubimini_msgs__srv__SetSensorConfiguration_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rokubimini_msgs__srv__SetSensorConfiguration_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rokubimini_msgs__srv__SetSensorConfiguration_Request__fini(&data[i - 1]);
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
rokubimini_msgs__srv__SetSensorConfiguration_Request__Sequence__fini(rokubimini_msgs__srv__SetSensorConfiguration_Request__Sequence * array)
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
      rokubimini_msgs__srv__SetSensorConfiguration_Request__fini(&array->data[i]);
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

rokubimini_msgs__srv__SetSensorConfiguration_Request__Sequence *
rokubimini_msgs__srv__SetSensorConfiguration_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rokubimini_msgs__srv__SetSensorConfiguration_Request__Sequence * array = (rokubimini_msgs__srv__SetSensorConfiguration_Request__Sequence *)allocator.allocate(sizeof(rokubimini_msgs__srv__SetSensorConfiguration_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rokubimini_msgs__srv__SetSensorConfiguration_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rokubimini_msgs__srv__SetSensorConfiguration_Request__Sequence__destroy(rokubimini_msgs__srv__SetSensorConfiguration_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rokubimini_msgs__srv__SetSensorConfiguration_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rokubimini_msgs__srv__SetSensorConfiguration_Request__Sequence__are_equal(const rokubimini_msgs__srv__SetSensorConfiguration_Request__Sequence * lhs, const rokubimini_msgs__srv__SetSensorConfiguration_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rokubimini_msgs__srv__SetSensorConfiguration_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rokubimini_msgs__srv__SetSensorConfiguration_Request__Sequence__copy(
  const rokubimini_msgs__srv__SetSensorConfiguration_Request__Sequence * input,
  rokubimini_msgs__srv__SetSensorConfiguration_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rokubimini_msgs__srv__SetSensorConfiguration_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rokubimini_msgs__srv__SetSensorConfiguration_Request * data =
      (rokubimini_msgs__srv__SetSensorConfiguration_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rokubimini_msgs__srv__SetSensorConfiguration_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rokubimini_msgs__srv__SetSensorConfiguration_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rokubimini_msgs__srv__SetSensorConfiguration_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
rokubimini_msgs__srv__SetSensorConfiguration_Response__init(rokubimini_msgs__srv__SetSensorConfiguration_Response * msg)
{
  if (!msg) {
    return false;
  }
  // b
  return true;
}

void
rokubimini_msgs__srv__SetSensorConfiguration_Response__fini(rokubimini_msgs__srv__SetSensorConfiguration_Response * msg)
{
  if (!msg) {
    return;
  }
  // b
}

bool
rokubimini_msgs__srv__SetSensorConfiguration_Response__are_equal(const rokubimini_msgs__srv__SetSensorConfiguration_Response * lhs, const rokubimini_msgs__srv__SetSensorConfiguration_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // b
  if (lhs->b != rhs->b) {
    return false;
  }
  return true;
}

bool
rokubimini_msgs__srv__SetSensorConfiguration_Response__copy(
  const rokubimini_msgs__srv__SetSensorConfiguration_Response * input,
  rokubimini_msgs__srv__SetSensorConfiguration_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // b
  output->b = input->b;
  return true;
}

rokubimini_msgs__srv__SetSensorConfiguration_Response *
rokubimini_msgs__srv__SetSensorConfiguration_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rokubimini_msgs__srv__SetSensorConfiguration_Response * msg = (rokubimini_msgs__srv__SetSensorConfiguration_Response *)allocator.allocate(sizeof(rokubimini_msgs__srv__SetSensorConfiguration_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rokubimini_msgs__srv__SetSensorConfiguration_Response));
  bool success = rokubimini_msgs__srv__SetSensorConfiguration_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rokubimini_msgs__srv__SetSensorConfiguration_Response__destroy(rokubimini_msgs__srv__SetSensorConfiguration_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rokubimini_msgs__srv__SetSensorConfiguration_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rokubimini_msgs__srv__SetSensorConfiguration_Response__Sequence__init(rokubimini_msgs__srv__SetSensorConfiguration_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rokubimini_msgs__srv__SetSensorConfiguration_Response * data = NULL;

  if (size) {
    data = (rokubimini_msgs__srv__SetSensorConfiguration_Response *)allocator.zero_allocate(size, sizeof(rokubimini_msgs__srv__SetSensorConfiguration_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rokubimini_msgs__srv__SetSensorConfiguration_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rokubimini_msgs__srv__SetSensorConfiguration_Response__fini(&data[i - 1]);
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
rokubimini_msgs__srv__SetSensorConfiguration_Response__Sequence__fini(rokubimini_msgs__srv__SetSensorConfiguration_Response__Sequence * array)
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
      rokubimini_msgs__srv__SetSensorConfiguration_Response__fini(&array->data[i]);
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

rokubimini_msgs__srv__SetSensorConfiguration_Response__Sequence *
rokubimini_msgs__srv__SetSensorConfiguration_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rokubimini_msgs__srv__SetSensorConfiguration_Response__Sequence * array = (rokubimini_msgs__srv__SetSensorConfiguration_Response__Sequence *)allocator.allocate(sizeof(rokubimini_msgs__srv__SetSensorConfiguration_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rokubimini_msgs__srv__SetSensorConfiguration_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rokubimini_msgs__srv__SetSensorConfiguration_Response__Sequence__destroy(rokubimini_msgs__srv__SetSensorConfiguration_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rokubimini_msgs__srv__SetSensorConfiguration_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rokubimini_msgs__srv__SetSensorConfiguration_Response__Sequence__are_equal(const rokubimini_msgs__srv__SetSensorConfiguration_Response__Sequence * lhs, const rokubimini_msgs__srv__SetSensorConfiguration_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rokubimini_msgs__srv__SetSensorConfiguration_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rokubimini_msgs__srv__SetSensorConfiguration_Response__Sequence__copy(
  const rokubimini_msgs__srv__SetSensorConfiguration_Response__Sequence * input,
  rokubimini_msgs__srv__SetSensorConfiguration_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rokubimini_msgs__srv__SetSensorConfiguration_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rokubimini_msgs__srv__SetSensorConfiguration_Response * data =
      (rokubimini_msgs__srv__SetSensorConfiguration_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rokubimini_msgs__srv__SetSensorConfiguration_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rokubimini_msgs__srv__SetSensorConfiguration_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rokubimini_msgs__srv__SetSensorConfiguration_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
