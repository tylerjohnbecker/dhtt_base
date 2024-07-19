// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dhtt_msgs:msg/Resources.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__MSG__DETAIL__RESOURCES__STRUCT_H_
#define DHTT_MSGS__MSG__DETAIL__RESOURCES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'resource_state'
#include "dhtt_msgs/msg/detail/resource__struct.h"

// Struct defined in msg/Resources in the package dhtt_msgs.
typedef struct dhtt_msgs__msg__Resources
{
  dhtt_msgs__msg__Resource__Sequence resource_state;
} dhtt_msgs__msg__Resources;

// Struct for a sequence of dhtt_msgs__msg__Resources.
typedef struct dhtt_msgs__msg__Resources__Sequence
{
  dhtt_msgs__msg__Resources * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__msg__Resources__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DHTT_MSGS__MSG__DETAIL__RESOURCES__STRUCT_H_
