// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dhtt_msgs:msg/Resource.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__MSG__DETAIL__RESOURCE__STRUCT_H_
#define DHTT_MSGS__MSG__DETAIL__RESOURCE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'GRIPPER'.
enum
{
  dhtt_msgs__msg__Resource__GRIPPER = 1
};

/// Constant 'HEAD'.
enum
{
  dhtt_msgs__msg__Resource__HEAD = 2
};

/// Constant 'BASE'.
enum
{
  dhtt_msgs__msg__Resource__BASE = 3
};

/// Constant 'EXCLUSIVE'.
enum
{
  dhtt_msgs__msg__Resource__EXCLUSIVE = 1
};

/// Constant 'SHARED'.
enum
{
  dhtt_msgs__msg__Resource__SHARED = 2
};

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/Resource in the package dhtt_msgs.
typedef struct dhtt_msgs__msg__Resource
{
  int8_t type;
  bool locked;
  int16_t owners;
  rosidl_runtime_c__String name;
  int8_t channel;
} dhtt_msgs__msg__Resource;

// Struct for a sequence of dhtt_msgs__msg__Resource.
typedef struct dhtt_msgs__msg__Resource__Sequence
{
  dhtt_msgs__msg__Resource * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__msg__Resource__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DHTT_MSGS__MSG__DETAIL__RESOURCE__STRUCT_H_
