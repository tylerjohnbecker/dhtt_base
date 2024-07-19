// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dhtt_msgs:srv/FetchInfo.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__SRV__DETAIL__FETCH_INFO__STRUCT_H_
#define DHTT_MSGS__SRV__DETAIL__FETCH_INFO__STRUCT_H_

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

// Struct defined in srv/FetchInfo in the package dhtt_msgs.
typedef struct dhtt_msgs__srv__FetchInfo_Request
{
  rosidl_runtime_c__String__Sequence keys;
} dhtt_msgs__srv__FetchInfo_Request;

// Struct for a sequence of dhtt_msgs__srv__FetchInfo_Request.
typedef struct dhtt_msgs__srv__FetchInfo_Request__Sequence
{
  dhtt_msgs__srv__FetchInfo_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__srv__FetchInfo_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'information_pairs'
#include "dhtt_msgs/msg/detail/pair__struct.h"

// Struct defined in srv/FetchInfo in the package dhtt_msgs.
typedef struct dhtt_msgs__srv__FetchInfo_Response
{
  dhtt_msgs__msg__Pair__Sequence information_pairs;
} dhtt_msgs__srv__FetchInfo_Response;

// Struct for a sequence of dhtt_msgs__srv__FetchInfo_Response.
typedef struct dhtt_msgs__srv__FetchInfo_Response__Sequence
{
  dhtt_msgs__srv__FetchInfo_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__srv__FetchInfo_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DHTT_MSGS__SRV__DETAIL__FETCH_INFO__STRUCT_H_
