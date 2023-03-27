// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rokubimini_msgs:srv/GetSensorConfiguration.idl
// generated code does not contain a copyright notice

#ifndef ROKUBIMINI_MSGS__SRV__DETAIL__GET_SENSOR_CONFIGURATION__FUNCTIONS_H_
#define ROKUBIMINI_MSGS__SRV__DETAIL__GET_SENSOR_CONFIGURATION__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "rokubimini_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "rokubimini_msgs/srv/detail/get_sensor_configuration__struct.h"

/// Initialize srv/GetSensorConfiguration message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rokubimini_msgs__srv__GetSensorConfiguration_Request
 * )) before or use
 * rokubimini_msgs__srv__GetSensorConfiguration_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
bool
rokubimini_msgs__srv__GetSensorConfiguration_Request__init(rokubimini_msgs__srv__GetSensorConfiguration_Request * msg);

/// Finalize srv/GetSensorConfiguration message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
void
rokubimini_msgs__srv__GetSensorConfiguration_Request__fini(rokubimini_msgs__srv__GetSensorConfiguration_Request * msg);

/// Create srv/GetSensorConfiguration message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rokubimini_msgs__srv__GetSensorConfiguration_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
rokubimini_msgs__srv__GetSensorConfiguration_Request *
rokubimini_msgs__srv__GetSensorConfiguration_Request__create();

/// Destroy srv/GetSensorConfiguration message.
/**
 * It calls
 * rokubimini_msgs__srv__GetSensorConfiguration_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
void
rokubimini_msgs__srv__GetSensorConfiguration_Request__destroy(rokubimini_msgs__srv__GetSensorConfiguration_Request * msg);

/// Check for srv/GetSensorConfiguration message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
bool
rokubimini_msgs__srv__GetSensorConfiguration_Request__are_equal(const rokubimini_msgs__srv__GetSensorConfiguration_Request * lhs, const rokubimini_msgs__srv__GetSensorConfiguration_Request * rhs);

/// Copy a srv/GetSensorConfiguration message.
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
rokubimini_msgs__srv__GetSensorConfiguration_Request__copy(
  const rokubimini_msgs__srv__GetSensorConfiguration_Request * input,
  rokubimini_msgs__srv__GetSensorConfiguration_Request * output);

/// Initialize array of srv/GetSensorConfiguration messages.
/**
 * It allocates the memory for the number of elements and calls
 * rokubimini_msgs__srv__GetSensorConfiguration_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
bool
rokubimini_msgs__srv__GetSensorConfiguration_Request__Sequence__init(rokubimini_msgs__srv__GetSensorConfiguration_Request__Sequence * array, size_t size);

/// Finalize array of srv/GetSensorConfiguration messages.
/**
 * It calls
 * rokubimini_msgs__srv__GetSensorConfiguration_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
void
rokubimini_msgs__srv__GetSensorConfiguration_Request__Sequence__fini(rokubimini_msgs__srv__GetSensorConfiguration_Request__Sequence * array);

/// Create array of srv/GetSensorConfiguration messages.
/**
 * It allocates the memory for the array and calls
 * rokubimini_msgs__srv__GetSensorConfiguration_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
rokubimini_msgs__srv__GetSensorConfiguration_Request__Sequence *
rokubimini_msgs__srv__GetSensorConfiguration_Request__Sequence__create(size_t size);

/// Destroy array of srv/GetSensorConfiguration messages.
/**
 * It calls
 * rokubimini_msgs__srv__GetSensorConfiguration_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
void
rokubimini_msgs__srv__GetSensorConfiguration_Request__Sequence__destroy(rokubimini_msgs__srv__GetSensorConfiguration_Request__Sequence * array);

/// Check for srv/GetSensorConfiguration message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
bool
rokubimini_msgs__srv__GetSensorConfiguration_Request__Sequence__are_equal(const rokubimini_msgs__srv__GetSensorConfiguration_Request__Sequence * lhs, const rokubimini_msgs__srv__GetSensorConfiguration_Request__Sequence * rhs);

/// Copy an array of srv/GetSensorConfiguration messages.
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
rokubimini_msgs__srv__GetSensorConfiguration_Request__Sequence__copy(
  const rokubimini_msgs__srv__GetSensorConfiguration_Request__Sequence * input,
  rokubimini_msgs__srv__GetSensorConfiguration_Request__Sequence * output);

/// Initialize srv/GetSensorConfiguration message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rokubimini_msgs__srv__GetSensorConfiguration_Response
 * )) before or use
 * rokubimini_msgs__srv__GetSensorConfiguration_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
bool
rokubimini_msgs__srv__GetSensorConfiguration_Response__init(rokubimini_msgs__srv__GetSensorConfiguration_Response * msg);

/// Finalize srv/GetSensorConfiguration message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
void
rokubimini_msgs__srv__GetSensorConfiguration_Response__fini(rokubimini_msgs__srv__GetSensorConfiguration_Response * msg);

/// Create srv/GetSensorConfiguration message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rokubimini_msgs__srv__GetSensorConfiguration_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
rokubimini_msgs__srv__GetSensorConfiguration_Response *
rokubimini_msgs__srv__GetSensorConfiguration_Response__create();

/// Destroy srv/GetSensorConfiguration message.
/**
 * It calls
 * rokubimini_msgs__srv__GetSensorConfiguration_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
void
rokubimini_msgs__srv__GetSensorConfiguration_Response__destroy(rokubimini_msgs__srv__GetSensorConfiguration_Response * msg);

/// Check for srv/GetSensorConfiguration message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
bool
rokubimini_msgs__srv__GetSensorConfiguration_Response__are_equal(const rokubimini_msgs__srv__GetSensorConfiguration_Response * lhs, const rokubimini_msgs__srv__GetSensorConfiguration_Response * rhs);

/// Copy a srv/GetSensorConfiguration message.
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
rokubimini_msgs__srv__GetSensorConfiguration_Response__copy(
  const rokubimini_msgs__srv__GetSensorConfiguration_Response * input,
  rokubimini_msgs__srv__GetSensorConfiguration_Response * output);

/// Initialize array of srv/GetSensorConfiguration messages.
/**
 * It allocates the memory for the number of elements and calls
 * rokubimini_msgs__srv__GetSensorConfiguration_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
bool
rokubimini_msgs__srv__GetSensorConfiguration_Response__Sequence__init(rokubimini_msgs__srv__GetSensorConfiguration_Response__Sequence * array, size_t size);

/// Finalize array of srv/GetSensorConfiguration messages.
/**
 * It calls
 * rokubimini_msgs__srv__GetSensorConfiguration_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
void
rokubimini_msgs__srv__GetSensorConfiguration_Response__Sequence__fini(rokubimini_msgs__srv__GetSensorConfiguration_Response__Sequence * array);

/// Create array of srv/GetSensorConfiguration messages.
/**
 * It allocates the memory for the array and calls
 * rokubimini_msgs__srv__GetSensorConfiguration_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
rokubimini_msgs__srv__GetSensorConfiguration_Response__Sequence *
rokubimini_msgs__srv__GetSensorConfiguration_Response__Sequence__create(size_t size);

/// Destroy array of srv/GetSensorConfiguration messages.
/**
 * It calls
 * rokubimini_msgs__srv__GetSensorConfiguration_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
void
rokubimini_msgs__srv__GetSensorConfiguration_Response__Sequence__destroy(rokubimini_msgs__srv__GetSensorConfiguration_Response__Sequence * array);

/// Check for srv/GetSensorConfiguration message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rokubimini_msgs
bool
rokubimini_msgs__srv__GetSensorConfiguration_Response__Sequence__are_equal(const rokubimini_msgs__srv__GetSensorConfiguration_Response__Sequence * lhs, const rokubimini_msgs__srv__GetSensorConfiguration_Response__Sequence * rhs);

/// Copy an array of srv/GetSensorConfiguration messages.
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
rokubimini_msgs__srv__GetSensorConfiguration_Response__Sequence__copy(
  const rokubimini_msgs__srv__GetSensorConfiguration_Response__Sequence * input,
  rokubimini_msgs__srv__GetSensorConfiguration_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROKUBIMINI_MSGS__SRV__DETAIL__GET_SENSOR_CONFIGURATION__FUNCTIONS_H_
