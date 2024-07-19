// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dhtt_msgs:msg/Subtree.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dhtt_msgs/msg/detail/subtree__rosidl_typesupport_introspection_c.h"
#include "dhtt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dhtt_msgs/msg/detail/subtree__functions.h"
#include "dhtt_msgs/msg/detail/subtree__struct.h"


// Include directives for member types
// Member `tree_nodes`
#include "dhtt_msgs/msg/node.h"
// Member `tree_nodes`
#include "dhtt_msgs/msg/detail/node__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Subtree__rosidl_typesupport_introspection_c__Subtree_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dhtt_msgs__msg__Subtree__init(message_memory);
}

void Subtree__rosidl_typesupport_introspection_c__Subtree_fini_function(void * message_memory)
{
  dhtt_msgs__msg__Subtree__fini(message_memory);
}

size_t Subtree__rosidl_typesupport_introspection_c__size_function__Node__tree_nodes(
  const void * untyped_member)
{
  const dhtt_msgs__msg__Node__Sequence * member =
    (const dhtt_msgs__msg__Node__Sequence *)(untyped_member);
  return member->size;
}

const void * Subtree__rosidl_typesupport_introspection_c__get_const_function__Node__tree_nodes(
  const void * untyped_member, size_t index)
{
  const dhtt_msgs__msg__Node__Sequence * member =
    (const dhtt_msgs__msg__Node__Sequence *)(untyped_member);
  return &member->data[index];
}

void * Subtree__rosidl_typesupport_introspection_c__get_function__Node__tree_nodes(
  void * untyped_member, size_t index)
{
  dhtt_msgs__msg__Node__Sequence * member =
    (dhtt_msgs__msg__Node__Sequence *)(untyped_member);
  return &member->data[index];
}

bool Subtree__rosidl_typesupport_introspection_c__resize_function__Node__tree_nodes(
  void * untyped_member, size_t size)
{
  dhtt_msgs__msg__Node__Sequence * member =
    (dhtt_msgs__msg__Node__Sequence *)(untyped_member);
  dhtt_msgs__msg__Node__Sequence__fini(member);
  return dhtt_msgs__msg__Node__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember Subtree__rosidl_typesupport_introspection_c__Subtree_message_member_array[5] = {
  {
    "tree_nodes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__msg__Subtree, tree_nodes),  // bytes offset in struct
    NULL,  // default value
    Subtree__rosidl_typesupport_introspection_c__size_function__Node__tree_nodes,  // size() function pointer
    Subtree__rosidl_typesupport_introspection_c__get_const_function__Node__tree_nodes,  // get_const(index) function pointer
    Subtree__rosidl_typesupport_introspection_c__get_function__Node__tree_nodes,  // get(index) function pointer
    Subtree__rosidl_typesupport_introspection_c__resize_function__Node__tree_nodes  // resize(index) function pointer
  },
  {
    "tree_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__msg__Subtree, tree_status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "max_tree_depth",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__msg__Subtree, max_tree_depth),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "max_tree_width",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__msg__Subtree, max_tree_width),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "task_completion_percent",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__msg__Subtree, task_completion_percent),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Subtree__rosidl_typesupport_introspection_c__Subtree_message_members = {
  "dhtt_msgs__msg",  // message namespace
  "Subtree",  // message name
  5,  // number of fields
  sizeof(dhtt_msgs__msg__Subtree),
  Subtree__rosidl_typesupport_introspection_c__Subtree_message_member_array,  // message members
  Subtree__rosidl_typesupport_introspection_c__Subtree_init_function,  // function to initialize message memory (memory has to be allocated)
  Subtree__rosidl_typesupport_introspection_c__Subtree_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Subtree__rosidl_typesupport_introspection_c__Subtree_message_type_support_handle = {
  0,
  &Subtree__rosidl_typesupport_introspection_c__Subtree_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dhtt_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, msg, Subtree)() {
  Subtree__rosidl_typesupport_introspection_c__Subtree_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, msg, Node)();
  if (!Subtree__rosidl_typesupport_introspection_c__Subtree_message_type_support_handle.typesupport_identifier) {
    Subtree__rosidl_typesupport_introspection_c__Subtree_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Subtree__rosidl_typesupport_introspection_c__Subtree_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
