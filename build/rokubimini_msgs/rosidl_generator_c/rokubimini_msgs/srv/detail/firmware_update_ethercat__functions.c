// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rokubimini_msgs:srv/FirmwareUpdateEthercat.idl
// generated code does not contain a copyright notice
#include "rokubimini_msgs/srv/detail/firmware_update_ethercat__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `file_name`
// Member `file_path`
#include "rosidl_runtime_c/string_functions.h"

bool
rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__init(rokubimini_msgs__srv__FirmwareUpdateEthercat_Request * msg)
{
  if (!msg) {
    return false;
  }
  // file_name
  if (!rosidl_runtime_c__String__init(&msg->file_name)) {
    rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__fini(msg);
    return false;
  }
  // file_path
  if (!rosidl_runtime_c__String__init(&msg->file_path)) {
    rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__fini(msg);
    return false;
  }
  // password
  return true;
}

void
rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__fini(rokubimini_msgs__srv__FirmwareUpdateEthercat_Request * msg)
{
  if (!msg) {
    return;
  }
  // file_name
  rosidl_runtime_c__String__fini(&msg->file_name);
  // file_path
  rosidl_runtime_c__String__fini(&msg->file_path);
  // password
}

bool
rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__are_equal(const rokubimini_msgs__srv__FirmwareUpdateEthercat_Request * lhs, const rokubimini_msgs__srv__FirmwareUpdateEthercat_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // file_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->file_name), &(rhs->file_name)))
  {
    return false;
  }
  // file_path
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->file_path), &(rhs->file_path)))
  {
    return false;
  }
  // password
  if (lhs->password != rhs->password) {
    return false;
  }
  return true;
}

bool
rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__copy(
  const rokubimini_msgs__srv__FirmwareUpdateEthercat_Request * input,
  rokubimini_msgs__srv__FirmwareUpdateEthercat_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // file_name
  if (!rosidl_runtime_c__String__copy(
      &(input->file_name), &(output->file_name)))
  {
    return false;
  }
  // file_path
  if (!rosidl_runtime_c__String__copy(
      &(input->file_path), &(output->file_path)))
  {
    return false;
  }
  // password
  output->password = input->password;
  return true;
}

rokubimini_msgs__srv__FirmwareUpdateEthercat_Request *
rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rokubimini_msgs__srv__FirmwareUpdateEthercat_Request * msg = (rokubimini_msgs__srv__FirmwareUpdateEthercat_Request *)allocator.allocate(sizeof(rokubimini_msgs__srv__FirmwareUpdateEthercat_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rokubimini_msgs__srv__FirmwareUpdateEthercat_Request));
  bool success = rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__destroy(rokubimini_msgs__srv__FirmwareUpdateEthercat_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__Sequence__init(rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rokubimini_msgs__srv__FirmwareUpdateEthercat_Request * data = NULL;

  if (size) {
    data = (rokubimini_msgs__srv__FirmwareUpdateEthercat_Request *)allocator.zero_allocate(size, sizeof(rokubimini_msgs__srv__FirmwareUpdateEthercat_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__fini(&data[i - 1]);
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
rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__Sequence__fini(rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__Sequence * array)
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
      rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__fini(&array->data[i]);
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

rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__Sequence *
rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__Sequence * array = (rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__Sequence *)allocator.allocate(sizeof(rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__Sequence__destroy(rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__Sequence__are_equal(const rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__Sequence * lhs, const rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__Sequence__copy(
  const rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__Sequence * input,
  rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rokubimini_msgs__srv__FirmwareUpdateEthercat_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rokubimini_msgs__srv__FirmwareUpdateEthercat_Request * data =
      (rokubimini_msgs__srv__FirmwareUpdateEthercat_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rokubimini_msgs__srv__FirmwareUpdateEthercat_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__init(rokubimini_msgs__srv__FirmwareUpdateEthercat_Response * msg)
{
  if (!msg) {
    return false;
  }
  // result
  return true;
}

void
rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__fini(rokubimini_msgs__srv__FirmwareUpdateEthercat_Response * msg)
{
  if (!msg) {
    return;
  }
  // result
}

bool
rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__are_equal(const rokubimini_msgs__srv__FirmwareUpdateEthercat_Response * lhs, const rokubimini_msgs__srv__FirmwareUpdateEthercat_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // result
  if (lhs->result != rhs->result) {
    return false;
  }
  return true;
}

bool
rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__copy(
  const rokubimini_msgs__srv__FirmwareUpdateEthercat_Response * input,
  rokubimini_msgs__srv__FirmwareUpdateEthercat_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // result
  output->result = input->result;
  return true;
}

rokubimini_msgs__srv__FirmwareUpdateEthercat_Response *
rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rokubimini_msgs__srv__FirmwareUpdateEthercat_Response * msg = (rokubimini_msgs__srv__FirmwareUpdateEthercat_Response *)allocator.allocate(sizeof(rokubimini_msgs__srv__FirmwareUpdateEthercat_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rokubimini_msgs__srv__FirmwareUpdateEthercat_Response));
  bool success = rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__destroy(rokubimini_msgs__srv__FirmwareUpdateEthercat_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__Sequence__init(rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rokubimini_msgs__srv__FirmwareUpdateEthercat_Response * data = NULL;

  if (size) {
    data = (rokubimini_msgs__srv__FirmwareUpdateEthercat_Response *)allocator.zero_allocate(size, sizeof(rokubimini_msgs__srv__FirmwareUpdateEthercat_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__fini(&data[i - 1]);
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
rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__Sequence__fini(rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__Sequence * array)
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
      rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__fini(&array->data[i]);
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

rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__Sequence *
rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__Sequence * array = (rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__Sequence *)allocator.allocate(sizeof(rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__Sequence__destroy(rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__Sequence__are_equal(const rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__Sequence * lhs, const rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__Sequence__copy(
  const rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__Sequence * input,
  rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rokubimini_msgs__srv__FirmwareUpdateEthercat_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rokubimini_msgs__srv__FirmwareUpdateEthercat_Response * data =
      (rokubimini_msgs__srv__FirmwareUpdateEthercat_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rokubimini_msgs__srv__FirmwareUpdateEthercat_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
