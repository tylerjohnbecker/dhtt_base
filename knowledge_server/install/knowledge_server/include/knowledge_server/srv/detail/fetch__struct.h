// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from knowledge_server:srv/Fetch.idl
// generated code does not contain a copyright notice

#ifndef KNOWLEDGE_SERVER__SRV__DETAIL__FETCH__STRUCT_H_
#define KNOWLEDGE_SERVER__SRV__DETAIL__FETCH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'keys'
#include "rosidl_runtime_c/string.h"

// Struct defined in srv/Fetch in the package knowledge_server.
typedef struct knowledge_server__srv__Fetch_Request
{
  rosidl_runtime_c__String__Sequence keys;
} knowledge_server__srv__Fetch_Request;

// Struct for a sequence of knowledge_server__srv__Fetch_Request.
typedef struct knowledge_server__srv__Fetch_Request__Sequence
{
  knowledge_server__srv__Fetch_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} knowledge_server__srv__Fetch_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'pairs'
#include "knowledge_server/msg/detail/pair__struct.h"

// Struct defined in srv/Fetch in the package knowledge_server.
typedef struct knowledge_server__srv__Fetch_Response
{
  knowledge_server__msg__Pair__Sequence pairs;
} knowledge_server__srv__Fetch_Response;

// Struct for a sequence of knowledge_server__srv__Fetch_Response.
typedef struct knowledge_server__srv__Fetch_Response__Sequence
{
  knowledge_server__srv__Fetch_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} knowledge_server__srv__Fetch_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KNOWLEDGE_SERVER__SRV__DETAIL__FETCH__STRUCT_H_
