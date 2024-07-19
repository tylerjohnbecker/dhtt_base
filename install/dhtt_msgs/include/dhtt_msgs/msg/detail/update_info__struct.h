// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dhtt_msgs:msg/UpdateInfo.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__MSG__DETAIL__UPDATE_INFO__STRUCT_H_
#define DHTT_MSGS__MSG__DETAIL__UPDATE_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'updates'
#include "dhtt_msgs/msg/detail/pair__struct.h"

// Struct defined in msg/UpdateInfo in the package dhtt_msgs.
typedef struct dhtt_msgs__msg__UpdateInfo
{
  dhtt_msgs__msg__Pair__Sequence updates;
} dhtt_msgs__msg__UpdateInfo;

// Struct for a sequence of dhtt_msgs__msg__UpdateInfo.
typedef struct dhtt_msgs__msg__UpdateInfo__Sequence
{
  dhtt_msgs__msg__UpdateInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__msg__UpdateInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DHTT_MSGS__MSG__DETAIL__UPDATE_INFO__STRUCT_H_
