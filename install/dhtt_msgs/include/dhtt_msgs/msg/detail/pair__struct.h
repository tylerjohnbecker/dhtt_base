// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dhtt_msgs:msg/Pair.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__MSG__DETAIL__PAIR__STRUCT_H_
#define DHTT_MSGS__MSG__DETAIL__PAIR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'key'
// Member 'value'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/Pair in the package dhtt_msgs.
typedef struct dhtt_msgs__msg__Pair
{
  rosidl_runtime_c__String key;
  rosidl_runtime_c__String value;
} dhtt_msgs__msg__Pair;

// Struct for a sequence of dhtt_msgs__msg__Pair.
typedef struct dhtt_msgs__msg__Pair__Sequence
{
  dhtt_msgs__msg__Pair * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__msg__Pair__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DHTT_MSGS__MSG__DETAIL__PAIR__STRUCT_H_
