// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dhtt_msgs:srv/FetchRequest.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__SRV__DETAIL__FETCH_REQUEST__STRUCT_H_
#define DHTT_MSGS__SRV__DETAIL__FETCH_REQUEST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'ROOT'.
enum
{
  dhtt_msgs__srv__FetchRequest_Request__ROOT = 1
};

/// Constant 'AND'.
enum
{
  dhtt_msgs__srv__FetchRequest_Request__AND = 2
};

/// Constant 'THEN'.
enum
{
  dhtt_msgs__srv__FetchRequest_Request__THEN = 3
};

/// Constant 'OR'.
enum
{
  dhtt_msgs__srv__FetchRequest_Request__OR = 4
};

/// Constant 'BEHAVIOR'.
enum
{
  dhtt_msgs__srv__FetchRequest_Request__BEHAVIOR = 5
};

// Include directives for member types
// Member 'common_name'
// Member 'node_name'
#include "rosidl_runtime_c/string.h"

// Struct defined in srv/FetchRequest in the package dhtt_msgs.
typedef struct dhtt_msgs__srv__FetchRequest_Request
{
  bool return_full_subtree;
  rosidl_runtime_c__String common_name;
  rosidl_runtime_c__String node_name;
  int8_t node_type;
} dhtt_msgs__srv__FetchRequest_Request;

// Struct for a sequence of dhtt_msgs__srv__FetchRequest_Request.
typedef struct dhtt_msgs__srv__FetchRequest_Request__Sequence
{
  dhtt_msgs__srv__FetchRequest_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__srv__FetchRequest_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'error_msg'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'found_subtrees'
#include "dhtt_msgs/msg/detail/subtree__struct.h"

// Struct defined in srv/FetchRequest in the package dhtt_msgs.
typedef struct dhtt_msgs__srv__FetchRequest_Response
{
  bool success;
  rosidl_runtime_c__String error_msg;
  dhtt_msgs__msg__Subtree__Sequence found_subtrees;
} dhtt_msgs__srv__FetchRequest_Response;

// Struct for a sequence of dhtt_msgs__srv__FetchRequest_Response.
typedef struct dhtt_msgs__srv__FetchRequest_Response__Sequence
{
  dhtt_msgs__srv__FetchRequest_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__srv__FetchRequest_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DHTT_MSGS__SRV__DETAIL__FETCH_REQUEST__STRUCT_H_
