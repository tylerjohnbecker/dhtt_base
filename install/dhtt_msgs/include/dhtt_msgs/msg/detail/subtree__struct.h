// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dhtt_msgs:msg/Subtree.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__MSG__DETAIL__SUBTREE__STRUCT_H_
#define DHTT_MSGS__MSG__DETAIL__SUBTREE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'tree_nodes'
#include "dhtt_msgs/msg/detail/node__struct.h"

// Struct defined in msg/Subtree in the package dhtt_msgs.
typedef struct dhtt_msgs__msg__Subtree
{
  dhtt_msgs__msg__Node__Sequence tree_nodes;
  int8_t tree_status;
  int32_t max_tree_depth;
  int32_t max_tree_width;
  float task_completion_percent;
} dhtt_msgs__msg__Subtree;

// Struct for a sequence of dhtt_msgs__msg__Subtree.
typedef struct dhtt_msgs__msg__Subtree__Sequence
{
  dhtt_msgs__msg__Subtree * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__msg__Subtree__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DHTT_MSGS__MSG__DETAIL__SUBTREE__STRUCT_H_
