// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from knowledge_server:msg/Update.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "knowledge_server/msg/detail/update__rosidl_typesupport_introspection_c.h"
#include "knowledge_server/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "knowledge_server/msg/detail/update__functions.h"
#include "knowledge_server/msg/detail/update__struct.h"


// Include directives for member types
// Member `updated_pairs`
#include "knowledge_server/msg/pair.h"
// Member `updated_pairs`
#include "knowledge_server/msg/detail/pair__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Update__rosidl_typesupport_introspection_c__Update_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  knowledge_server__msg__Update__init(message_memory);
}

void Update__rosidl_typesupport_introspection_c__Update_fini_function(void * message_memory)
{
  knowledge_server__msg__Update__fini(message_memory);
}

size_t Update__rosidl_typesupport_introspection_c__size_function__Pair__updated_pairs(
  const void * untyped_member)
{
  const knowledge_server__msg__Pair__Sequence * member =
    (const knowledge_server__msg__Pair__Sequence *)(untyped_member);
  return member->size;
}

const void * Update__rosidl_typesupport_introspection_c__get_const_function__Pair__updated_pairs(
  const void * untyped_member, size_t index)
{
  const knowledge_server__msg__Pair__Sequence * member =
    (const knowledge_server__msg__Pair__Sequence *)(untyped_member);
  return &member->data[index];
}

void * Update__rosidl_typesupport_introspection_c__get_function__Pair__updated_pairs(
  void * untyped_member, size_t index)
{
  knowledge_server__msg__Pair__Sequence * member =
    (knowledge_server__msg__Pair__Sequence *)(untyped_member);
  return &member->data[index];
}

bool Update__rosidl_typesupport_introspection_c__resize_function__Pair__updated_pairs(
  void * untyped_member, size_t size)
{
  knowledge_server__msg__Pair__Sequence * member =
    (knowledge_server__msg__Pair__Sequence *)(untyped_member);
  knowledge_server__msg__Pair__Sequence__fini(member);
  return knowledge_server__msg__Pair__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember Update__rosidl_typesupport_introspection_c__Update_message_member_array[1] = {
  {
    "updated_pairs",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(knowledge_server__msg__Update, updated_pairs),  // bytes offset in struct
    NULL,  // default value
    Update__rosidl_typesupport_introspection_c__size_function__Pair__updated_pairs,  // size() function pointer
    Update__rosidl_typesupport_introspection_c__get_const_function__Pair__updated_pairs,  // get_const(index) function pointer
    Update__rosidl_typesupport_introspection_c__get_function__Pair__updated_pairs,  // get(index) function pointer
    Update__rosidl_typesupport_introspection_c__resize_function__Pair__updated_pairs  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Update__rosidl_typesupport_introspection_c__Update_message_members = {
  "knowledge_server__msg",  // message namespace
  "Update",  // message name
  1,  // number of fields
  sizeof(knowledge_server__msg__Update),
  Update__rosidl_typesupport_introspection_c__Update_message_member_array,  // message members
  Update__rosidl_typesupport_introspection_c__Update_init_function,  // function to initialize message memory (memory has to be allocated)
  Update__rosidl_typesupport_introspection_c__Update_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Update__rosidl_typesupport_introspection_c__Update_message_type_support_handle = {
  0,
  &Update__rosidl_typesupport_introspection_c__Update_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_knowledge_server
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, knowledge_server, msg, Update)() {
  Update__rosidl_typesupport_introspection_c__Update_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, knowledge_server, msg, Pair)();
  if (!Update__rosidl_typesupport_introspection_c__Update_message_type_support_handle.typesupport_identifier) {
    Update__rosidl_typesupport_introspection_c__Update_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Update__rosidl_typesupport_introspection_c__Update_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
