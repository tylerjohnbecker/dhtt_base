// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from knowledge_server:msg/Pair.idl
// generated code does not contain a copyright notice

#ifndef KNOWLEDGE_SERVER__MSG__DETAIL__PAIR__STRUCT_H_
#define KNOWLEDGE_SERVER__MSG__DETAIL__PAIR__STRUCT_H_

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

// Struct defined in msg/Pair in the package knowledge_server.
typedef struct knowledge_server__msg__Pair
{
  rosidl_runtime_c__String key;
  rosidl_runtime_c__String value;
} knowledge_server__msg__Pair;

// Struct for a sequence of knowledge_server__msg__Pair.
typedef struct knowledge_server__msg__Pair__Sequence
{
  knowledge_server__msg__Pair * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} knowledge_server__msg__Pair__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KNOWLEDGE_SERVER__MSG__DETAIL__PAIR__STRUCT_H_
