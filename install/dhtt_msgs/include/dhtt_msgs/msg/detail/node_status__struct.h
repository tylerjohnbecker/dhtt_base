// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dhtt_msgs:msg/NodeStatus.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__MSG__DETAIL__NODE_STATUS__STRUCT_H_
#define DHTT_MSGS__MSG__DETAIL__NODE_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'WAITING'.
enum
{
  dhtt_msgs__msg__NodeStatus__WAITING = 1
};

/// Constant 'ACTIVE'.
enum
{
  dhtt_msgs__msg__NodeStatus__ACTIVE = 2
};

/// Constant 'WORKING'.
enum
{
  dhtt_msgs__msg__NodeStatus__WORKING = 3
};

/// Constant 'DONE'.
enum
{
  dhtt_msgs__msg__NodeStatus__DONE = 4
};

/// Constant 'MUTATING'.
enum
{
  dhtt_msgs__msg__NodeStatus__MUTATING = 5
};

// Struct defined in msg/NodeStatus in the package dhtt_msgs.
typedef struct dhtt_msgs__msg__NodeStatus
{
  int8_t state;
} dhtt_msgs__msg__NodeStatus;

// Struct for a sequence of dhtt_msgs__msg__NodeStatus.
typedef struct dhtt_msgs__msg__NodeStatus__Sequence
{
  dhtt_msgs__msg__NodeStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__msg__NodeStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DHTT_MSGS__MSG__DETAIL__NODE_STATUS__STRUCT_H_
