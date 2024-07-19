// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dhtt_msgs:msg/UpdateInfo.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dhtt_msgs/msg/detail/update_info__rosidl_typesupport_introspection_c.h"
#include "dhtt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dhtt_msgs/msg/detail/update_info__functions.h"
#include "dhtt_msgs/msg/detail/update_info__struct.h"


// Include directives for member types
// Member `updates`
#include "dhtt_msgs/msg/pair.h"
// Member `updates`
#include "dhtt_msgs/msg/detail/pair__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void UpdateInfo__rosidl_typesupport_introspection_c__UpdateInfo_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dhtt_msgs__msg__UpdateInfo__init(message_memory);
}

void UpdateInfo__rosidl_typesupport_introspection_c__UpdateInfo_fini_function(void * message_memory)
{
  dhtt_msgs__msg__UpdateInfo__fini(message_memory);
}

size_t UpdateInfo__rosidl_typesupport_introspection_c__size_function__Pair__updates(
  const void * untyped_member)
{
  const dhtt_msgs__msg__Pair__Sequence * member =
    (const dhtt_msgs__msg__Pair__Sequence *)(untyped_member);
  return member->size;
}

const void * UpdateInfo__rosidl_typesupport_introspection_c__get_const_function__Pair__updates(
  const void * untyped_member, size_t index)
{
  const dhtt_msgs__msg__Pair__Sequence * member =
    (const dhtt_msgs__msg__Pair__Sequence *)(untyped_member);
  return &member->data[index];
}

void * UpdateInfo__rosidl_typesupport_introspection_c__get_function__Pair__updates(
  void * untyped_member, size_t index)
{
  dhtt_msgs__msg__Pair__Sequence * member =
    (dhtt_msgs__msg__Pair__Sequence *)(untyped_member);
  return &member->data[index];
}

bool UpdateInfo__rosidl_typesupport_introspection_c__resize_function__Pair__updates(
  void * untyped_member, size_t size)
{
  dhtt_msgs__msg__Pair__Sequence * member =
    (dhtt_msgs__msg__Pair__Sequence *)(untyped_member);
  dhtt_msgs__msg__Pair__Sequence__fini(member);
  return dhtt_msgs__msg__Pair__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember UpdateInfo__rosidl_typesupport_introspection_c__UpdateInfo_message_member_array[1] = {
  {
    "updates",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__msg__UpdateInfo, updates),  // bytes offset in struct
    NULL,  // default value
    UpdateInfo__rosidl_typesupport_introspection_c__size_function__Pair__updates,  // size() function pointer
    UpdateInfo__rosidl_typesupport_introspection_c__get_const_function__Pair__updates,  // get_const(index) function pointer
    UpdateInfo__rosidl_typesupport_introspection_c__get_function__Pair__updates,  // get(index) function pointer
    UpdateInfo__rosidl_typesupport_introspection_c__resize_function__Pair__updates  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers UpdateInfo__rosidl_typesupport_introspection_c__UpdateInfo_message_members = {
  "dhtt_msgs__msg",  // message namespace
  "UpdateInfo",  // message name
  1,  // number of fields
  sizeof(dhtt_msgs__msg__UpdateInfo),
  UpdateInfo__rosidl_typesupport_introspection_c__UpdateInfo_message_member_array,  // message members
  UpdateInfo__rosidl_typesupport_introspection_c__UpdateInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  UpdateInfo__rosidl_typesupport_introspection_c__UpdateInfo_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t UpdateInfo__rosidl_typesupport_introspection_c__UpdateInfo_message_type_support_handle = {
  0,
  &UpdateInfo__rosidl_typesupport_introspection_c__UpdateInfo_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dhtt_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, msg, UpdateInfo)() {
  UpdateInfo__rosidl_typesupport_introspection_c__UpdateInfo_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, msg, Pair)();
  if (!UpdateInfo__rosidl_typesupport_introspection_c__UpdateInfo_message_type_support_handle.typesupport_identifier) {
    UpdateInfo__rosidl_typesupport_introspection_c__UpdateInfo_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &UpdateInfo__rosidl_typesupport_introspection_c__UpdateInfo_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
