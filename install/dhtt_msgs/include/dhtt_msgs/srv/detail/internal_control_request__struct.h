// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dhtt_msgs:srv/InternalControlRequest.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__SRV__DETAIL__INTERNAL_CONTROL_REQUEST__STRUCT_H_
#define DHTT_MSGS__SRV__DETAIL__INTERNAL_CONTROL_REQUEST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'START'.
enum
{
  dhtt_msgs__srv__InternalControlRequest_Request__START = 1
};

/// Constant 'GRACEFULSTOP'.
enum
{
  dhtt_msgs__srv__InternalControlRequest_Request__GRACEFULSTOP = 2
};

/// Constant 'IMMEDIATESTOP'.
enum
{
  dhtt_msgs__srv__InternalControlRequest_Request__IMMEDIATESTOP = 3
};

// Struct defined in srv/InternalControlRequest in the package dhtt_msgs.
typedef struct dhtt_msgs__srv__InternalControlRequest_Request
{
  int8_t control_code;
} dhtt_msgs__srv__InternalControlRequest_Request;

// Struct for a sequence of dhtt_msgs__srv__InternalControlRequest_Request.
typedef struct dhtt_msgs__srv__InternalControlRequest_Request__Sequence
{
  dhtt_msgs__srv__InternalControlRequest_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__srv__InternalControlRequest_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'error_msg'
#include "rosidl_runtime_c/string.h"

// Struct defined in srv/InternalControlRequest in the package dhtt_msgs.
typedef struct dhtt_msgs__srv__InternalControlRequest_Response
{
  bool success;
  rosidl_runtime_c__String error_msg;
} dhtt_msgs__srv__InternalControlRequest_Response;

// Struct for a sequence of dhtt_msgs__srv__InternalControlRequest_Response.
typedef struct dhtt_msgs__srv__InternalControlRequest_Response__Sequence
{
  dhtt_msgs__srv__InternalControlRequest_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__srv__InternalControlRequest_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DHTT_MSGS__SRV__DETAIL__INTERNAL_CONTROL_REQUEST__STRUCT_H_
