// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dhtt_msgs:srv/ModifyRequest.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__SRV__DETAIL__MODIFY_REQUEST__STRUCT_H_
#define DHTT_MSGS__SRV__DETAIL__MODIFY_REQUEST__STRUCT_H_

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
  dhtt_msgs__srv__ModifyRequest_Request__ADD = 0
};

/// Constant 'ADD_FROM_FILE'.
enum
{
  dhtt_msgs__srv__ModifyRequest_Request__ADD_FROM_FILE = 1
};

/// Constant 'REMOVE'.
enum
{
  dhtt_msgs__srv__ModifyRequest_Request__REMOVE = 2
};

/// Constant 'PARAM_UPDATE'.
enum
{
  dhtt_msgs__srv__ModifyRequest_Request__PARAM_UPDATE = 3
};

/// Constant 'MUTATE'.
enum
{
  dhtt_msgs__srv__ModifyRequest_Request__MUTATE = 4
};

// Include directives for member types
// Member 'to_modify'
// Member 'to_add'
// Member 'params'
// Member 'mutate_type'
#include "rosidl_runtime_c/string.h"
// Member 'add_node'
#include "dhtt_msgs/msg/detail/node__struct.h"

// Struct defined in srv/ModifyRequest in the package dhtt_msgs.
typedef struct dhtt_msgs__srv__ModifyRequest_Request
{
  int8_t type;
  rosidl_runtime_c__String__Sequence to_modify;
  dhtt_msgs__msg__Node add_node;
  rosidl_runtime_c__String to_add;
  rosidl_runtime_c__String__Sequence params;
  rosidl_runtime_c__String mutate_type;
} dhtt_msgs__srv__ModifyRequest_Request;

// Struct for a sequence of dhtt_msgs__srv__ModifyRequest_Request.
typedef struct dhtt_msgs__srv__ModifyRequest_Request__Sequence
{
  dhtt_msgs__srv__ModifyRequest_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__srv__ModifyRequest_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'error_msg'
// Member 'added_nodes'
// Member 'removed_nodes'
// already included above
// #include "rosidl_runtime_c/string.h"

// Struct defined in srv/ModifyRequest in the package dhtt_msgs.
typedef struct dhtt_msgs__srv__ModifyRequest_Response
{
  bool success;
  rosidl_runtime_c__String error_msg;
  rosidl_runtime_c__String__Sequence added_nodes;
  rosidl_runtime_c__String__Sequence removed_nodes;
} dhtt_msgs__srv__ModifyRequest_Response;

// Struct for a sequence of dhtt_msgs__srv__ModifyRequest_Response.
typedef struct dhtt_msgs__srv__ModifyRequest_Response__Sequence
{
  dhtt_msgs__srv__ModifyRequest_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__srv__ModifyRequest_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DHTT_MSGS__SRV__DETAIL__MODIFY_REQUEST__STRUCT_H_
