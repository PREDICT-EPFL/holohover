// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rokubimini_msgs:msg/Reading.idl
// generated code does not contain a copyright notice

#ifndef ROKUBIMINI_MSGS__MSG__DETAIL__READING__FUNCTIONS_H_
#define ROKUBIMINI_MSGS__MSG__DETAIL__READING__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "rokubimini_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "rokubimini_msgs/msg/detail/reading__struct.h"

/// Initialize msg/Reading message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rokubimini_msgs__msg__Reading
 * )) before or use
 * rokubimini_msgs__msg__Reading__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
bool
rokubimini_msgs__msg__Reading__init(rokubimini_msgs__msg__Reading * msg);

/// Finalize msg/Reading message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
void
rokubimini_msgs__msg__Reading__fini(rokubimini_msgs__msg__Reading * msg);

/// Create msg/Reading message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rokubimini_msgs__msg__Reading__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
rokubimini_msgs__msg__Reading *
rokubimini_msgs__msg__Reading__create();

/// Destroy msg/Reading message.
/**
 * It calls
 * rokubimini_msgs__msg__Reading__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
void
rokubimini_msgs__msg__Reading__destroy(rokubimini_msgs__msg__Reading * msg);

/// Check for msg/Reading message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
bool
rokubimini_msgs__msg__Reading__are_equal(const rokubimini_msgs__msg__Reading * lhs, const rokubimini_msgs__msg__Reading * rhs);

/// Copy a msg/Reading message.
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
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
bool
rokubimini_msgs__msg__Reading__copy(
  const rokubimini_msgs__msg__Reading * input,
  rokubimini_msgs__msg__Reading * output);

/// Initialize array of msg/Reading messages.
/**
 * It allocates the memory for the number of elements and calls
 * rokubimini_msgs__msg__Reading__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
bool
rokubimini_msgs__msg__Reading__Sequence__init(rokubimini_msgs__msg__Reading__Sequence * array, size_t size);

/// Finalize array of msg/Reading messages.
/**
 * It calls
 * rokubimini_msgs__msg__Reading__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
void
rokubimini_msgs__msg__Reading__Sequence__fini(rokubimini_msgs__msg__Reading__Sequence * array);

/// Create array of msg/Reading messages.
/**
 * It allocates the memory for the array and calls
 * rokubimini_msgs__msg__Reading__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
rokubimini_msgs__msg__Reading__Sequence *
rokubimini_msgs__msg__Reading__Sequence__create(size_t size);

/// Destroy array of msg/Reading messages.
/**
 * It calls
 * rokubimini_msgs__msg__Reading__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
void
rokubimini_msgs__msg__Reading__Sequence__destroy(rokubimini_msgs__msg__Reading__Sequence * array);

/// Check for msg/Reading message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
bool
rokubimini_msgs__msg__Reading__Sequence__are_equal(const rokubimini_msgs__msg__Reading__Sequence * lhs, const rokubimini_msgs__msg__Reading__Sequence * rhs);

/// Copy an array of msg/Reading messages.
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
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
bool
rokubimini_msgs__msg__Reading__Sequence__copy(
  const rokubimini_msgs__msg__Reading__Sequence * input,
  rokubimini_msgs__msg__Reading__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROKUBIMINI_MSGS__MSG__DETAIL__READING__FUNCTIONS_H_
