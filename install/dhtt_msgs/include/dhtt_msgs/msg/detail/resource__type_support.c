// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dhtt_msgs:msg/Resource.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dhtt_msgs/msg/detail/resource__rosidl_typesupport_introspection_c.h"
#include "dhtt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dhtt_msgs/msg/detail/resource__functions.h"
#include "dhtt_msgs/msg/detail/resource__struct.h"


// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Resource__rosidl_typesupport_introspection_c__Resource_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dhtt_msgs__msg__Resource__init(message_memory);
}

void Resource__rosidl_typesupport_introspection_c__Resource_fini_function(void * message_memory)
{
  dhtt_msgs__msg__Resource__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Resource__rosidl_typesupport_introspection_c__Resource_message_member_array[5] = {
  {
    "type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__msg__Resource, type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "locked",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__msg__Resource, locked),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "owners",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__msg__Resource, owners),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__msg__Resource, name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "channel",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__msg__Resource, channel),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Resource__rosidl_typesupport_introspection_c__Resource_message_members = {
  "dhtt_msgs__msg",  // message namespace
  "Resource",  // message name
  5,  // number of fields
  sizeof(dhtt_msgs__msg__Resource),
  Resource__rosidl_typesupport_introspection_c__Resource_message_member_array,  // message members
  Resource__rosidl_typesupport_introspection_c__Resource_init_function,  // function to initialize message memory (memory has to be allocated)
  Resource__rosidl_typesupport_introspection_c__Resource_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Resource__rosidl_typesupport_introspection_c__Resource_message_type_support_handle = {
  0,
  &Resource__rosidl_typesupport_introspection_c__Resource_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dhtt_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, msg, Resource)() {
  if (!Resource__rosidl_typesupport_introspection_c__Resource_message_type_support_handle.typesupport_identifier) {
    Resource__rosidl_typesupport_introspection_c__Resource_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Resource__rosidl_typesupport_introspection_c__Resource_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
