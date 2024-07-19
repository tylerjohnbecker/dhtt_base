// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dhtt_msgs:msg/Node.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dhtt_msgs/msg/detail/node__rosidl_typesupport_introspection_c.h"
#include "dhtt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dhtt_msgs/msg/detail/node__functions.h"
#include "dhtt_msgs/msg/detail/node__struct.h"


// Include directives for member types
// Member `head`
#include "std_msgs/msg/header.h"
// Member `head`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `node_name`
// Member `parent_name`
// Member `child_name`
// Member `params`
// Member `plugin_name`
#include "rosidl_runtime_c/string_functions.h"
// Member `children`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `owned_resources`
#include "dhtt_msgs/msg/resource.h"
// Member `owned_resources`
#include "dhtt_msgs/msg/detail/resource__rosidl_typesupport_introspection_c.h"
// Member `node_status`
#include "dhtt_msgs/msg/node_status.h"
// Member `node_status`
#include "dhtt_msgs/msg/detail/node_status__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Node__rosidl_typesupport_introspection_c__Node_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dhtt_msgs__msg__Node__init(message_memory);
}

void Node__rosidl_typesupport_introspection_c__Node_fini_function(void * message_memory)
{
  dhtt_msgs__msg__Node__fini(message_memory);
}

size_t Node__rosidl_typesupport_introspection_c__size_function__Resource__owned_resources(
  const void * untyped_member)
{
  const dhtt_msgs__msg__Resource__Sequence * member =
    (const dhtt_msgs__msg__Resource__Sequence *)(untyped_member);
  return member->size;
}

const void * Node__rosidl_typesupport_introspection_c__get_const_function__Resource__owned_resources(
  const void * untyped_member, size_t index)
{
  const dhtt_msgs__msg__Resource__Sequence * member =
    (const dhtt_msgs__msg__Resource__Sequence *)(untyped_member);
  return &member->data[index];
}

void * Node__rosidl_typesupport_introspection_c__get_function__Resource__owned_resources(
  void * untyped_member, size_t index)
{
  dhtt_msgs__msg__Resource__Sequence * member =
    (dhtt_msgs__msg__Resource__Sequence *)(untyped_member);
  return &member->data[index];
}

bool Node__rosidl_typesupport_introspection_c__resize_function__Resource__owned_resources(
  void * untyped_member, size_t size)
{
  dhtt_msgs__msg__Resource__Sequence * member =
    (dhtt_msgs__msg__Resource__Sequence *)(untyped_member);
  dhtt_msgs__msg__Resource__Sequence__fini(member);
  return dhtt_msgs__msg__Resource__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember Node__rosidl_typesupport_introspection_c__Node_message_member_array[11] = {
  {
    "head",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__msg__Node, head),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "node_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__msg__Node, node_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "parent",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__msg__Node, parent),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "parent_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__msg__Node, parent_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "children",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__msg__Node, children),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "child_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__msg__Node, child_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "params",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__msg__Node, params),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__msg__Node, type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "plugin_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__msg__Node, plugin_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "owned_resources",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__msg__Node, owned_resources),  // bytes offset in struct
    NULL,  // default value
    Node__rosidl_typesupport_introspection_c__size_function__Resource__owned_resources,  // size() function pointer
    Node__rosidl_typesupport_introspection_c__get_const_function__Resource__owned_resources,  // get_const(index) function pointer
    Node__rosidl_typesupport_introspection_c__get_function__Resource__owned_resources,  // get(index) function pointer
    Node__rosidl_typesupport_introspection_c__resize_function__Resource__owned_resources  // resize(index) function pointer
  },
  {
    "node_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__msg__Node, node_status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Node__rosidl_typesupport_introspection_c__Node_message_members = {
  "dhtt_msgs__msg",  // message namespace
  "Node",  // message name
  11,  // number of fields
  sizeof(dhtt_msgs__msg__Node),
  Node__rosidl_typesupport_introspection_c__Node_message_member_array,  // message members
  Node__rosidl_typesupport_introspection_c__Node_init_function,  // function to initialize message memory (memory has to be allocated)
  Node__rosidl_typesupport_introspection_c__Node_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Node__rosidl_typesupport_introspection_c__Node_message_type_support_handle = {
  0,
  &Node__rosidl_typesupport_introspection_c__Node_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dhtt_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, msg, Node)() {
  Node__rosidl_typesupport_introspection_c__Node_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  Node__rosidl_typesupport_introspection_c__Node_message_member_array[9].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, msg, Resource)();
  Node__rosidl_typesupport_introspection_c__Node_message_member_array[10].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, msg, NodeStatus)();
  if (!Node__rosidl_typesupport_introspection_c__Node_message_type_support_handle.typesupport_identifier) {
    Node__rosidl_typesupport_introspection_c__Node_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Node__rosidl_typesupport_introspection_c__Node_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
