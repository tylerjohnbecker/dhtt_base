// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from dhtt_msgs:srv/InternalServiceRegistration.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__SRV__DETAIL__INTERNAL_SERVICE_REGISTRATION__FUNCTIONS_H_
#define DHTT_MSGS__SRV__DETAIL__INTERNAL_SERVICE_REGISTRATION__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "dhtt_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "dhtt_msgs/srv/detail/internal_service_registration__struct.h"

/// Initialize srv/InternalServiceRegistration message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * dhtt_msgs__srv__InternalServiceRegistration_Request
 * )) before or use
 * dhtt_msgs__srv__InternalServiceRegistration_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
bool
dhtt_msgs__srv__InternalServiceRegistration_Request__init(dhtt_msgs__srv__InternalServiceRegistration_Request * msg);

/// Finalize srv/InternalServiceRegistration message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
void
dhtt_msgs__srv__InternalServiceRegistration_Request__fini(dhtt_msgs__srv__InternalServiceRegistration_Request * msg);

/// Create srv/InternalServiceRegistration message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * dhtt_msgs__srv__InternalServiceRegistration_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
dhtt_msgs__srv__InternalServiceRegistration_Request *
dhtt_msgs__srv__InternalServiceRegistration_Request__create();

/// Destroy srv/InternalServiceRegistration message.
/**
 * It calls
 * dhtt_msgs__srv__InternalServiceRegistration_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
void
dhtt_msgs__srv__InternalServiceRegistration_Request__destroy(dhtt_msgs__srv__InternalServiceRegistration_Request * msg);

/// Check for srv/InternalServiceRegistration message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
bool
dhtt_msgs__srv__InternalServiceRegistration_Request__are_equal(const dhtt_msgs__srv__InternalServiceRegistration_Request * lhs, const dhtt_msgs__srv__InternalServiceRegistration_Request * rhs);

/// Copy a srv/InternalServiceRegistration message.
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
dhtt_msgs__srv__InternalServiceRegistration_Request__copy(
  const dhtt_msgs__srv__InternalServiceRegistration_Request * input,
  dhtt_msgs__srv__InternalServiceRegistration_Request * output);

/// Initialize array of srv/InternalServiceRegistration messages.
/**
 * It allocates the memory for the number of elements and calls
 * dhtt_msgs__srv__InternalServiceRegistration_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
bool
dhtt_msgs__srv__InternalServiceRegistration_Request__Sequence__init(dhtt_msgs__srv__InternalServiceRegistration_Request__Sequence * array, size_t size);

/// Finalize array of srv/InternalServiceRegistration messages.
/**
 * It calls
 * dhtt_msgs__srv__InternalServiceRegistration_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
void
dhtt_msgs__srv__InternalServiceRegistration_Request__Sequence__fini(dhtt_msgs__srv__InternalServiceRegistration_Request__Sequence * array);

/// Create array of srv/InternalServiceRegistration messages.
/**
 * It allocates the memory for the array and calls
 * dhtt_msgs__srv__InternalServiceRegistration_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
dhtt_msgs__srv__InternalServiceRegistration_Request__Sequence *
dhtt_msgs__srv__InternalServiceRegistration_Request__Sequence__create(size_t size);

/// Destroy array of srv/InternalServiceRegistration messages.
/**
 * It calls
 * dhtt_msgs__srv__InternalServiceRegistration_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
void
dhtt_msgs__srv__InternalServiceRegistration_Request__Sequence__destroy(dhtt_msgs__srv__InternalServiceRegistration_Request__Sequence * array);

/// Check for srv/InternalServiceRegistration message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
bool
dhtt_msgs__srv__InternalServiceRegistration_Request__Sequence__are_equal(const dhtt_msgs__srv__InternalServiceRegistration_Request__Sequence * lhs, const dhtt_msgs__srv__InternalServiceRegistration_Request__Sequence * rhs);

/// Copy an array of srv/InternalServiceRegistration messages.
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
dhtt_msgs__srv__InternalServiceRegistration_Request__Sequence__copy(
  const dhtt_msgs__srv__InternalServiceRegistration_Request__Sequence * input,
  dhtt_msgs__srv__InternalServiceRegistration_Request__Sequence * output);

/// Initialize srv/InternalServiceRegistration message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * dhtt_msgs__srv__InternalServiceRegistration_Response
 * )) before or use
 * dhtt_msgs__srv__InternalServiceRegistration_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
bool
dhtt_msgs__srv__InternalServiceRegistration_Response__init(dhtt_msgs__srv__InternalServiceRegistration_Response * msg);

/// Finalize srv/InternalServiceRegistration message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
void
dhtt_msgs__srv__InternalServiceRegistration_Response__fini(dhtt_msgs__srv__InternalServiceRegistration_Response * msg);

/// Create srv/InternalServiceRegistration message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * dhtt_msgs__srv__InternalServiceRegistration_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
dhtt_msgs__srv__InternalServiceRegistration_Response *
dhtt_msgs__srv__InternalServiceRegistration_Response__create();

/// Destroy srv/InternalServiceRegistration message.
/**
 * It calls
 * dhtt_msgs__srv__InternalServiceRegistration_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
void
dhtt_msgs__srv__InternalServiceRegistration_Response__destroy(dhtt_msgs__srv__InternalServiceRegistration_Response * msg);

/// Check for srv/InternalServiceRegistration message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
bool
dhtt_msgs__srv__InternalServiceRegistration_Response__are_equal(const dhtt_msgs__srv__InternalServiceRegistration_Response * lhs, const dhtt_msgs__srv__InternalServiceRegistration_Response * rhs);

/// Copy a srv/InternalServiceRegistration message.
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
dhtt_msgs__srv__InternalServiceRegistration_Response__copy(
  const dhtt_msgs__srv__InternalServiceRegistration_Response * input,
  dhtt_msgs__srv__InternalServiceRegistration_Response * output);

/// Initialize array of srv/InternalServiceRegistration messages.
/**
 * It allocates the memory for the number of elements and calls
 * dhtt_msgs__srv__InternalServiceRegistration_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
bool
dhtt_msgs__srv__InternalServiceRegistration_Response__Sequence__init(dhtt_msgs__srv__InternalServiceRegistration_Response__Sequence * array, size_t size);

/// Finalize array of srv/InternalServiceRegistration messages.
/**
 * It calls
 * dhtt_msgs__srv__InternalServiceRegistration_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
void
dhtt_msgs__srv__InternalServiceRegistration_Response__Sequence__fini(dhtt_msgs__srv__InternalServiceRegistration_Response__Sequence * array);

/// Create array of srv/InternalServiceRegistration messages.
/**
 * It allocates the memory for the array and calls
 * dhtt_msgs__srv__InternalServiceRegistration_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
dhtt_msgs__srv__InternalServiceRegistration_Response__Sequence *
dhtt_msgs__srv__InternalServiceRegistration_Response__Sequence__create(size_t size);

/// Destroy array of srv/InternalServiceRegistration messages.
/**
 * It calls
 * dhtt_msgs__srv__InternalServiceRegistration_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
void
dhtt_msgs__srv__InternalServiceRegistration_Response__Sequence__destroy(dhtt_msgs__srv__InternalServiceRegistration_Response__Sequence * array);

/// Check for srv/InternalServiceRegistration message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dhtt_msgs
bool
dhtt_msgs__srv__InternalServiceRegistration_Response__Sequence__are_equal(const dhtt_msgs__srv__InternalServiceRegistration_Response__Sequence * lhs, const dhtt_msgs__srv__InternalServiceRegistration_Response__Sequence * rhs);

/// Copy an array of srv/InternalServiceRegistration messages.
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
dhtt_msgs__srv__InternalServiceRegistration_Response__Sequence__copy(
  const dhtt_msgs__srv__InternalServiceRegistration_Response__Sequence * input,
  dhtt_msgs__srv__InternalServiceRegistration_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // DHTT_MSGS__SRV__DETAIL__INTERNAL_SERVICE_REGISTRATION__FUNCTIONS_H_
