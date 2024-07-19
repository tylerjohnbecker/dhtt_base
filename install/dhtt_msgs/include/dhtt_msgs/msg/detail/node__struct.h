// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dhtt_msgs:msg/Node.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__MSG__DETAIL__NODE__STRUCT_H_
#define DHTT_MSGS__MSG__DETAIL__NODE__STRUCT_H_

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
  dhtt_msgs__msg__Node__ROOT = 0
};

/// Constant 'AND'.
enum
{
  dhtt_msgs__msg__Node__AND = 1
};

/// Constant 'THEN'.
enum
{
  dhtt_msgs__msg__Node__THEN = 2
};

/// Constant 'OR'.
enum
{
  dhtt_msgs__msg__Node__OR = 3
};

/// Constant 'BEHAVIOR'.
enum
{
  dhtt_msgs__msg__Node__BEHAVIOR = 4
};

// Include directives for member types
// Member 'head'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'node_name'
// Member 'parent_name'
// Member 'child_name'
// Member 'params'
// Member 'plugin_name'
#include "rosidl_runtime_c/string.h"
// Member 'children'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'owned_resources'
#include "dhtt_msgs/msg/detail/resource__struct.h"
// Member 'node_status'
#include "dhtt_msgs/msg/detail/node_status__struct.h"

// Struct defined in msg/Node in the package dhtt_msgs.
typedef struct dhtt_msgs__msg__Node
{
  std_msgs__msg__Header head;
  rosidl_runtime_c__String node_name;
  int32_t parent;
  rosidl_runtime_c__String parent_name;
  rosidl_runtime_c__int32__Sequence children;
  rosidl_runtime_c__String__Sequence child_name;
  rosidl_runtime_c__String__Sequence params;
  int8_t type;
  rosidl_runtime_c__String plugin_name;
  dhtt_msgs__msg__Resource__Sequence owned_resources;
  dhtt_msgs__msg__NodeStatus node_status;
} dhtt_msgs__msg__Node;

// Struct for a sequence of dhtt_msgs__msg__Node.
typedef struct dhtt_msgs__msg__Node__Sequence
{
  dhtt_msgs__msg__Node * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__msg__Node__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DHTT_MSGS__MSG__DETAIL__NODE__STRUCT_H_
