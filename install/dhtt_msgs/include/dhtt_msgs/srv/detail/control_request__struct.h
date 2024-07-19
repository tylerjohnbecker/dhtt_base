// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dhtt_msgs:srv/ControlRequest.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__SRV__DETAIL__CONTROL_REQUEST__STRUCT_H_
#define DHTT_MSGS__SRV__DETAIL__CONTROL_REQUEST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'STOP'.
enum
{
  dhtt_msgs__srv__ControlRequest_Request__STOP = 0
};

/// Constant 'START'.
enum
{
  dhtt_msgs__srv__ControlRequest_Request__START = 1
};

/// Constant 'SAVE_TO_FILE'.
enum
{
  dhtt_msgs__srv__ControlRequest_Request__SAVE_TO_FILE = 2
};

/// Constant 'RESET'.
enum
{
  dhtt_msgs__srv__ControlRequest_Request__RESET = 3
};

// Include directives for member types
// Member 'file_name'
#include "rosidl_runtime_c/string.h"

// Struct defined in srv/ControlRequest in the package dhtt_msgs.
typedef struct dhtt_msgs__srv__ControlRequest_Request
{
  int8_t type;
  rosidl_runtime_c__String file_name;
  bool interrupt;
} dhtt_msgs__srv__ControlRequest_Request;

// Struct for a sequence of dhtt_msgs__srv__ControlRequest_Request.
typedef struct dhtt_msgs__srv__ControlRequest_Request__Sequence
{
  dhtt_msgs__srv__ControlRequest_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__srv__ControlRequest_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'error_msg'
// already included above
// #include "rosidl_runtime_c/string.h"

// Struct defined in srv/ControlRequest in the package dhtt_msgs.
typedef struct dhtt_msgs__srv__ControlRequest_Response
{
  bool success;
  rosidl_runtime_c__String error_msg;
} dhtt_msgs__srv__ControlRequest_Response;

// Struct for a sequence of dhtt_msgs__srv__ControlRequest_Response.
typedef struct dhtt_msgs__srv__ControlRequest_Response__Sequence
{
  dhtt_msgs__srv__ControlRequest_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__srv__ControlRequest_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DHTT_MSGS__SRV__DETAIL__CONTROL_REQUEST__STRUCT_H_
