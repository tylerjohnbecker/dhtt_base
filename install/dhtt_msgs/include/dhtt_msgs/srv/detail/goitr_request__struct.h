// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dhtt_msgs:srv/GoitrRequest.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__SRV__DETAIL__GOITR_REQUEST__STRUCT_H_
#define DHTT_MSGS__SRV__DETAIL__GOITR_REQUEST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'GENERAL'.
enum
{
  dhtt_msgs__srv__GoitrRequest_Request__GENERAL = 0
};

// Include directives for member types
// Member 'params'
#include "rosidl_runtime_c/string.h"

// Struct defined in srv/GoitrRequest in the package dhtt_msgs.
typedef struct dhtt_msgs__srv__GoitrRequest_Request
{
  int8_t type;
  rosidl_runtime_c__String__Sequence params;
} dhtt_msgs__srv__GoitrRequest_Request;

// Struct for a sequence of dhtt_msgs__srv__GoitrRequest_Request.
typedef struct dhtt_msgs__srv__GoitrRequest_Request__Sequence
{
  dhtt_msgs__srv__GoitrRequest_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__srv__GoitrRequest_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'error_msgs'
// Member 'responses'
// already included above
// #include "rosidl_runtime_c/string.h"

// Struct defined in srv/GoitrRequest in the package dhtt_msgs.
typedef struct dhtt_msgs__srv__GoitrRequest_Response
{
  bool success;
  rosidl_runtime_c__String error_msgs;
  rosidl_runtime_c__String__Sequence responses;
} dhtt_msgs__srv__GoitrRequest_Response;

// Struct for a sequence of dhtt_msgs__srv__GoitrRequest_Response.
typedef struct dhtt_msgs__srv__GoitrRequest_Response__Sequence
{
  dhtt_msgs__srv__GoitrRequest_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__srv__GoitrRequest_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DHTT_MSGS__SRV__DETAIL__GOITR_REQUEST__STRUCT_H_
