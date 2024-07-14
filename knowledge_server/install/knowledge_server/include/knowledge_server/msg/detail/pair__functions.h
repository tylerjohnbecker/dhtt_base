// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from knowledge_server:msg/Pair.idl
// generated code does not contain a copyright notice

#ifndef KNOWLEDGE_SERVER__MSG__DETAIL__PAIR__FUNCTIONS_H_
#define KNOWLEDGE_SERVER__MSG__DETAIL__PAIR__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "knowledge_server/msg/rosidl_generator_c__visibility_control.h"

#include "knowledge_server/msg/detail/pair__struct.h"

/// Initialize msg/Pair message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * knowledge_server__msg__Pair
 * )) before or use
 * knowledge_server__msg__Pair__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_knowledge_server
bool
knowledge_server__msg__Pair__init(knowledge_server__msg__Pair * msg);

/// Finalize msg/Pair message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_knowledge_server
void
knowledge_server__msg__Pair__fini(knowledge_server__msg__Pair * msg);

/// Create msg/Pair message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * knowledge_server__msg__Pair__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_knowledge_server
knowledge_server__msg__Pair *
knowledge_server__msg__Pair__create();

/// Destroy msg/Pair message.
/**
 * It calls
 * knowledge_server__msg__Pair__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_knowledge_server
void
knowledge_server__msg__Pair__destroy(knowledge_server__msg__Pair * msg);

/// Check for msg/Pair message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_knowledge_server
bool
knowledge_server__msg__Pair__are_equal(const knowledge_server__msg__Pair * lhs, const knowledge_server__msg__Pair * rhs);

/// Copy a msg/Pair message.
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
ROSIDL_GENERATOR_C_PUBLIC_knowledge_server
bool
knowledge_server__msg__Pair__copy(
  const knowledge_server__msg__Pair * input,
  knowledge_server__msg__Pair * output);

/// Initialize array of msg/Pair messages.
/**
 * It allocates the memory for the number of elements and calls
 * knowledge_server__msg__Pair__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_knowledge_server
bool
knowledge_server__msg__Pair__Sequence__init(knowledge_server__msg__Pair__Sequence * array, size_t size);

/// Finalize array of msg/Pair messages.
/**
 * It calls
 * knowledge_server__msg__Pair__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_knowledge_server
void
knowledge_server__msg__Pair__Sequence__fini(knowledge_server__msg__Pair__Sequence * array);

/// Create array of msg/Pair messages.
/**
 * It allocates the memory for the array and calls
 * knowledge_server__msg__Pair__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_knowledge_server
knowledge_server__msg__Pair__Sequence *
knowledge_server__msg__Pair__Sequence__create(size_t size);

/// Destroy array of msg/Pair messages.
/**
 * It calls
 * knowledge_server__msg__Pair__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_knowledge_server
void
knowledge_server__msg__Pair__Sequence__destroy(knowledge_server__msg__Pair__Sequence * array);

/// Check for msg/Pair message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_knowledge_server
bool
knowledge_server__msg__Pair__Sequence__are_equal(const knowledge_server__msg__Pair__Sequence * lhs, const knowledge_server__msg__Pair__Sequence * rhs);

/// Copy an array of msg/Pair messages.
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
ROSIDL_GENERATOR_C_PUBLIC_knowledge_server
bool
knowledge_server__msg__Pair__Sequence__copy(
  const knowledge_server__msg__Pair__Sequence * input,
  knowledge_server__msg__Pair__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // KNOWLEDGE_SERVER__MSG__DETAIL__PAIR__FUNCTIONS_H_
