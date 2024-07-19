// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from dhtt_msgs:msg/UpdateInfo.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__MSG__DETAIL__UPDATE_INFO__FUNCTIONS_H_
#define DHTT_MSGS__MSG__DETAIL__UPDATE_INFO__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "dhtt_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "dhtt_msgs/msg/detail/update_info__struct.h"

/// Initialize msg/UpdateInfo message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * dhtt_msgs__msg__UpdateInfo
 * )) before or use
 * dhtt_msgs__msg__UpdateInfo__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
bool
dhtt_msgs__msg__UpdateInfo__init(dhtt_msgs__msg__UpdateInfo * msg);

/// Finalize msg/UpdateInfo message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
void
dhtt_msgs__msg__UpdateInfo__fini(dhtt_msgs__msg__UpdateInfo * msg);

/// Create msg/UpdateInfo message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * dhtt_msgs__msg__UpdateInfo__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
dhtt_msgs__msg__UpdateInfo *
dhtt_msgs__msg__UpdateInfo__create();

/// Destroy msg/UpdateInfo message.
/**
 * It calls
 * dhtt_msgs__msg__UpdateInfo__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
void
dhtt_msgs__msg__UpdateInfo__destroy(dhtt_msgs__msg__UpdateInfo * msg);

/// Check for msg/UpdateInfo message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
bool
dhtt_msgs__msg__UpdateInfo__are_equal(const dhtt_msgs__msg__UpdateInfo * lhs, const dhtt_msgs__msg__UpdateInfo * rhs);

/// Copy a msg/UpdateInfo message.
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
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
bool
dhtt_msgs__msg__UpdateInfo__copy(
  const dhtt_msgs__msg__UpdateInfo * input,
  dhtt_msgs__msg__UpdateInfo * output);

/// Initialize array of msg/UpdateInfo messages.
/**
 * It allocates the memory for the number of elements and calls
 * dhtt_msgs__msg__UpdateInfo__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
bool
dhtt_msgs__msg__UpdateInfo__Sequence__init(dhtt_msgs__msg__UpdateInfo__Sequence * array, size_t size);

/// Finalize array of msg/UpdateInfo messages.
/**
 * It calls
 * dhtt_msgs__msg__UpdateInfo__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
void
dhtt_msgs__msg__UpdateInfo__Sequence__fini(dhtt_msgs__msg__UpdateInfo__Sequence * array);

/// Create array of msg/UpdateInfo messages.
/**
 * It allocates the memory for the array and calls
 * dhtt_msgs__msg__UpdateInfo__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
dhtt_msgs__msg__UpdateInfo__Sequence *
dhtt_msgs__msg__UpdateInfo__Sequence__create(size_t size);

/// Destroy array of msg/UpdateInfo messages.
/**
 * It calls
 * dhtt_msgs__msg__UpdateInfo__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
void
dhtt_msgs__msg__UpdateInfo__Sequence__destroy(dhtt_msgs__msg__UpdateInfo__Sequence * array);

/// Check for msg/UpdateInfo message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
bool
dhtt_msgs__msg__UpdateInfo__Sequence__are_equal(const dhtt_msgs__msg__UpdateInfo__Sequence * lhs, const dhtt_msgs__msg__UpdateInfo__Sequence * rhs);

/// Copy an array of msg/UpdateInfo messages.
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
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
bool
dhtt_msgs__msg__UpdateInfo__Sequence__copy(
  const dhtt_msgs__msg__UpdateInfo__Sequence * input,
  dhtt_msgs__msg__UpdateInfo__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // DHTT_MSGS__MSG__DETAIL__UPDATE_INFO__FUNCTIONS_H_
