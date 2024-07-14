// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from knowledge_server:msg/Update.idl
// generated code does not contain a copyright notice

#ifndef KNOWLEDGE_SERVER__MSG__DETAIL__UPDATE__STRUCT_H_
#define KNOWLEDGE_SERVER__MSG__DETAIL__UPDATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'updated_pairs'
#include "knowledge_server/msg/detail/pair__struct.h"

// Struct defined in msg/Update in the package knowledge_server.
typedef struct knowledge_server__msg__Update
{
  knowledge_server__msg__Pair__Sequence updated_pairs;
} knowledge_server__msg__Update;

// Struct for a sequence of knowledge_server__msg__Update.
typedef struct knowledge_server__msg__Update__Sequence
{
  knowledge_server__msg__Update * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} knowledge_server__msg__Update__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KNOWLEDGE_SERVER__MSG__DETAIL__UPDATE__STRUCT_H_
