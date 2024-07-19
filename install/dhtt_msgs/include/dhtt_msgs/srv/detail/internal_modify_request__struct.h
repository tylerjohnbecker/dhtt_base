// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dhtt_msgs:srv/InternalModifyRequest.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__SRV__DETAIL__INTERNAL_MODIFY_REQUEST__STRUCT_H_
#define DHTT_MSGS__SRV__DETAIL__INTERNAL_MODIFY_REQUEST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'ADD'.
enum
{
  dhtt_msgs__srv__InternalModifyRequest_Request__ADD = 0
};

/// Constant 'REMOVE'.
enum
{
  dhtt_msgs__srv__InternalModifyRequest_Request__REMOVE = 1
};

/// Constant 'PARAM_UPDATE'.
enum
{
  dhtt_msgs__srv__InternalModifyRequest_Request__PARAM_UPDATE = 2
};

/// Constant 'MUTATE'.
enum
{
  dhtt_msgs__srv__InternalModifyRequest_Request__MUTATE = 3
};

// Include directives for member types
// Member 'node_name'
// Member 'plugin_name'
// Member 'params'
#include "rosidl_runtime_c/string.h"

// Struct defined in srv/InternalModifyRequest in the package dhtt_msgs.
typedef struct dhtt_msgs__srv__InternalModifyRequest_Request
{
  int8_t type;
  rosidl_runtime_c__String node_name;
  rosidl_runtime_c__String plugin_name;
  rosidl_runtime_c__String__Sequence params;
} dhtt_msgs__srv__InternalModifyRequest_Request;

// Struct for a sequence of dhtt_msgs__srv__InternalModifyRequest_Request.
typedef struct dhtt_msgs__srv__InternalModifyRequest_Request__Sequence
{
  dhtt_msgs__srv__InternalModifyRequest_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__srv__InternalModifyRequest_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'error_msg'
// already included above
// #include "rosidl_runtime_c/string.h"

// Struct defined in srv/InternalModifyRequest in the package dhtt_msgs.
typedef struct dhtt_msgs__srv__InternalModifyRequest_Response
{
  bool success;
  rosidl_runtime_c__String error_msg;
} dhtt_msgs__srv__InternalModifyRequest_Response;

// Struct for a sequence of dhtt_msgs__srv__InternalModifyRequest_Response.
typedef struct dhtt_msgs__srv__InternalModifyRequest_Response__Sequence
{
  dhtt_msgs__srv__InternalModifyRequest_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__srv__InternalModifyRequest_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DHTT_MSGS__SRV__DETAIL__INTERNAL_MODIFY_REQUEST__STRUCT_H_
