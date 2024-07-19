// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dhtt_msgs:srv/InternalServiceRegistration.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__SRV__DETAIL__INTERNAL_SERVICE_REGISTRATION__STRUCT_H_
#define DHTT_MSGS__SRV__DETAIL__INTERNAL_SERVICE_REGISTRATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'node_name'
#include "rosidl_runtime_c/string.h"

// Struct defined in srv/InternalServiceRegistration in the package dhtt_msgs.
typedef struct dhtt_msgs__srv__InternalServiceRegistration_Request
{
  rosidl_runtime_c__String node_name;
} dhtt_msgs__srv__InternalServiceRegistration_Request;

// Struct for a sequence of dhtt_msgs__srv__InternalServiceRegistration_Request.
typedef struct dhtt_msgs__srv__InternalServiceRegistration_Request__Sequence
{
  dhtt_msgs__srv__InternalServiceRegistration_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__srv__InternalServiceRegistration_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/InternalServiceRegistration in the package dhtt_msgs.
typedef struct dhtt_msgs__srv__InternalServiceRegistration_Response
{
  bool success;
} dhtt_msgs__srv__InternalServiceRegistration_Response;

// Struct for a sequence of dhtt_msgs__srv__InternalServiceRegistration_Response.
typedef struct dhtt_msgs__srv__InternalServiceRegistration_Response__Sequence
{
  dhtt_msgs__srv__InternalServiceRegistration_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__srv__InternalServiceRegistration_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DHTT_MSGS__SRV__DETAIL__INTERNAL_SERVICE_REGISTRATION__STRUCT_H_
