// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from holohover_msgs:msg/HolohoverControlStamped.idl
// generated code does not contain a copyright notice

#ifndef HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_CONTROL_STAMPED__FUNCTIONS_H_
#define HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_CONTROL_STAMPED__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "holohover_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "holohover_msgs/msg/detail/holohover_control_stamped__struct.h"

/// Initialize msg/HolohoverControlStamped message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * holohover_msgs__msg__HolohoverControlStamped
 * )) before or use
 * holohover_msgs__msg__HolohoverControlStamped__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_holohover_msgs
bool
holohover_msgs__msg__HolohoverControlStamped__init(holohover_msgs__msg__HolohoverControlStamped * msg);

/// Finalize msg/HolohoverControlStamped message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_holohover_msgs
void
holohover_msgs__msg__HolohoverControlStamped__fini(holohover_msgs__msg__HolohoverControlStamped * msg);

/// Create msg/HolohoverControlStamped message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * holohover_msgs__msg__HolohoverControlStamped__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_holohover_msgs
holohover_msgs__msg__HolohoverControlStamped *
holohover_msgs__msg__HolohoverControlStamped__create();

/// Destroy msg/HolohoverControlStamped message.
/**
 * It calls
 * holohover_msgs__msg__HolohoverControlStamped__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_holohover_msgs
void
holohover_msgs__msg__HolohoverControlStamped__destroy(holohover_msgs__msg__HolohoverControlStamped * msg);

/// Check for msg/HolohoverControlStamped message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_holohover_msgs
bool
holohover_msgs__msg__HolohoverControlStamped__are_equal(const holohover_msgs__msg__HolohoverControlStamped * lhs, const holohover_msgs__msg__HolohoverControlStamped * rhs);

/// Copy a msg/HolohoverControlStamped message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_holohover_msgs
bool
holohover_msgs__msg__HolohoverControlStamped__copy(
  const holohover_msgs__msg__HolohoverControlStamped * input,
  holohover_msgs__msg__HolohoverControlStamped * output);

/// Initialize array of msg/HolohoverControlStamped messages.
/**
 * It allocates the memory for the number of elements and calls
 * holohover_msgs__msg__HolohoverControlStamped__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_holohover_msgs
bool
holohover_msgs__msg__HolohoverControlStamped__Sequence__init(holohover_msgs__msg__HolohoverControlStamped__Sequence * array, size_t size);

/// Finalize array of msg/HolohoverControlStamped messages.
/**
 * It calls
 * holohover_msgs__msg__HolohoverControlStamped__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_holohover_msgs
void
holohover_msgs__msg__HolohoverControlStamped__Sequence__fini(holohover_msgs__msg__HolohoverControlStamped__Sequence * array);

/// Create array of msg/HolohoverControlStamped messages.
/**
 * It allocates the memory for the array and calls
 * holohover_msgs__msg__HolohoverControlStamped__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_holohover_msgs
holohover_msgs__msg__HolohoverControlStamped__Sequence *
holohover_msgs__msg__HolohoverControlStamped__Sequence__create(size_t size);

/// Destroy array of msg/HolohoverControlStamped messages.
/**
 * It calls
 * holohover_msgs__msg__HolohoverControlStamped__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_holohover_msgs
void
holohover_msgs__msg__HolohoverControlStamped__Sequence__destroy(holohover_msgs__msg__HolohoverControlStamped__Sequence * array);

/// Check for msg/HolohoverControlStamped message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_holohover_msgs
bool
holohover_msgs__msg__HolohoverControlStamped__Sequence__are_equal(const holohover_msgs__msg__HolohoverControlStamped__Sequence * lhs, const holohover_msgs__msg__HolohoverControlStamped__Sequence * rhs);

/// Copy an array of msg/HolohoverControlStamped messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_holohover_msgs
bool
holohover_msgs__msg__HolohoverControlStamped__Sequence__copy(
  const holohover_msgs__msg__HolohoverControlStamped__Sequence * input,
  holohover_msgs__msg__HolohoverControlStamped__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_CONTROL_STAMPED__FUNCTIONS_H_
