// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dhtt_msgs:action/Activation.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dhtt_msgs/action/detail/activation__rosidl_typesupport_introspection_c.h"
#include "dhtt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dhtt_msgs/action/detail/activation__functions.h"
#include "dhtt_msgs/action/detail/activation__struct.h"


// Include directives for member types
// Member `passed_resources`
// Member `granted_resources`
#include "dhtt_msgs/msg/resource.h"
// Member `passed_resources`
// Member `granted_resources`
#include "dhtt_msgs/msg/detail/resource__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Activation_Goal__rosidl_typesupport_introspection_c__Activation_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dhtt_msgs__action__Activation_Goal__init(message_memory);
}

void Activation_Goal__rosidl_typesupport_introspection_c__Activation_Goal_fini_function(void * message_memory)
{
  dhtt_msgs__action__Activation_Goal__fini(message_memory);
}

size_t Activation_Goal__rosidl_typesupport_introspection_c__size_function__Resource__passed_resources(
  const void * untyped_member)
{
  const dhtt_msgs__msg__Resource__Sequence * member =
    (const dhtt_msgs__msg__Resource__Sequence *)(untyped_member);
  return member->size;
}

const void * Activation_Goal__rosidl_typesupport_introspection_c__get_const_function__Resource__passed_resources(
  const void * untyped_member, size_t index)
{
  const dhtt_msgs__msg__Resource__Sequence * member =
    (const dhtt_msgs__msg__Resource__Sequence *)(untyped_member);
  return &member->data[index];
}

void * Activation_Goal__rosidl_typesupport_introspection_c__get_function__Resource__passed_resources(
  void * untyped_member, size_t index)
{
  dhtt_msgs__msg__Resource__Sequence * member =
    (dhtt_msgs__msg__Resource__Sequence *)(untyped_member);
  return &member->data[index];
}

bool Activation_Goal__rosidl_typesupport_introspection_c__resize_function__Resource__passed_resources(
  void * untyped_member, size_t size)
{
  dhtt_msgs__msg__Resource__Sequence * member =
    (dhtt_msgs__msg__Resource__Sequence *)(untyped_member);
  dhtt_msgs__msg__Resource__Sequence__fini(member);
  return dhtt_msgs__msg__Resource__Sequence__init(member, size);
}

size_t Activation_Goal__rosidl_typesupport_introspection_c__size_function__Resource__granted_resources(
  const void * untyped_member)
{
  const dhtt_msgs__msg__Resource__Sequence * member =
    (const dhtt_msgs__msg__Resource__Sequence *)(untyped_member);
  return member->size;
}

const void * Activation_Goal__rosidl_typesupport_introspection_c__get_const_function__Resource__granted_resources(
  const void * untyped_member, size_t index)
{
  const dhtt_msgs__msg__Resource__Sequence * member =
    (const dhtt_msgs__msg__Resource__Sequence *)(untyped_member);
  return &member->data[index];
}

void * Activation_Goal__rosidl_typesupport_introspection_c__get_function__Resource__granted_resources(
  void * untyped_member, size_t index)
{
  dhtt_msgs__msg__Resource__Sequence * member =
    (dhtt_msgs__msg__Resource__Sequence *)(untyped_member);
  return &member->data[index];
}

bool Activation_Goal__rosidl_typesupport_introspection_c__resize_function__Resource__granted_resources(
  void * untyped_member, size_t size)
{
  dhtt_msgs__msg__Resource__Sequence * member =
    (dhtt_msgs__msg__Resource__Sequence *)(untyped_member);
  dhtt_msgs__msg__Resource__Sequence__fini(member);
  return dhtt_msgs__msg__Resource__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember Activation_Goal__rosidl_typesupport_introspection_c__Activation_Goal_message_member_array[3] = {
  {
    "passed_resources",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__action__Activation_Goal, passed_resources),  // bytes offset in struct
    NULL,  // default value
    Activation_Goal__rosidl_typesupport_introspection_c__size_function__Resource__passed_resources,  // size() function pointer
    Activation_Goal__rosidl_typesupport_introspection_c__get_const_function__Resource__passed_resources,  // get_const(index) function pointer
    Activation_Goal__rosidl_typesupport_introspection_c__get_function__Resource__passed_resources,  // get(index) function pointer
    Activation_Goal__rosidl_typesupport_introspection_c__resize_function__Resource__passed_resources  // resize(index) function pointer
  },
  {
    "granted_resources",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__action__Activation_Goal, granted_resources),  // bytes offset in struct
    NULL,  // default value
    Activation_Goal__rosidl_typesupport_introspection_c__size_function__Resource__granted_resources,  // size() function pointer
    Activation_Goal__rosidl_typesupport_introspection_c__get_const_function__Resource__granted_resources,  // get_const(index) function pointer
    Activation_Goal__rosidl_typesupport_introspection_c__get_function__Resource__granted_resources,  // get(index) function pointer
    Activation_Goal__rosidl_typesupport_introspection_c__resize_function__Resource__granted_resources  // resize(index) function pointer
  },
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__action__Activation_Goal, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Activation_Goal__rosidl_typesupport_introspection_c__Activation_Goal_message_members = {
  "dhtt_msgs__action",  // message namespace
  "Activation_Goal",  // message name
  3,  // number of fields
  sizeof(dhtt_msgs__action__Activation_Goal),
  Activation_Goal__rosidl_typesupport_introspection_c__Activation_Goal_message_member_array,  // message members
  Activation_Goal__rosidl_typesupport_introspection_c__Activation_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  Activation_Goal__rosidl_typesupport_introspection_c__Activation_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Activation_Goal__rosidl_typesupport_introspection_c__Activation_Goal_message_type_support_handle = {
  0,
  &Activation_Goal__rosidl_typesupport_introspection_c__Activation_Goal_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dhtt_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, action, Activation_Goal)() {
  Activation_Goal__rosidl_typesupport_introspection_c__Activation_Goal_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, msg, Resource)();
  Activation_Goal__rosidl_typesupport_introspection_c__Activation_Goal_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, msg, Resource)();
  if (!Activation_Goal__rosidl_typesupport_introspection_c__Activation_Goal_message_type_support_handle.typesupport_identifier) {
    Activation_Goal__rosidl_typesupport_introspection_c__Activation_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Activation_Goal__rosidl_typesupport_introspection_c__Activation_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "dhtt_msgs/action/detail/activation__rosidl_typesupport_introspection_c.h"
// already included above
// #include "dhtt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "dhtt_msgs/action/detail/activation__functions.h"
// already included above
// #include "dhtt_msgs/action/detail/activation__struct.h"


// Include directives for member types
// Member `local_best_node`
#include "rosidl_runtime_c/string_functions.h"
// Member `requested_resources`
// Member `owned_resources`
// Member `released_resources`
// Member `passed_resources`
// already included above
// #include "dhtt_msgs/msg/resource.h"
// Member `requested_resources`
// Member `owned_resources`
// Member `released_resources`
// Member `passed_resources`
// already included above
// #include "dhtt_msgs/msg/detail/resource__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Activation_Result__rosidl_typesupport_introspection_c__Activation_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dhtt_msgs__action__Activation_Result__init(message_memory);
}

void Activation_Result__rosidl_typesupport_introspection_c__Activation_Result_fini_function(void * message_memory)
{
  dhtt_msgs__action__Activation_Result__fini(message_memory);
}

size_t Activation_Result__rosidl_typesupport_introspection_c__size_function__Resource__requested_resources(
  const void * untyped_member)
{
  const dhtt_msgs__msg__Resource__Sequence * member =
    (const dhtt_msgs__msg__Resource__Sequence *)(untyped_member);
  return member->size;
}

const void * Activation_Result__rosidl_typesupport_introspection_c__get_const_function__Resource__requested_resources(
  const void * untyped_member, size_t index)
{
  const dhtt_msgs__msg__Resource__Sequence * member =
    (const dhtt_msgs__msg__Resource__Sequence *)(untyped_member);
  return &member->data[index];
}

void * Activation_Result__rosidl_typesupport_introspection_c__get_function__Resource__requested_resources(
  void * untyped_member, size_t index)
{
  dhtt_msgs__msg__Resource__Sequence * member =
    (dhtt_msgs__msg__Resource__Sequence *)(untyped_member);
  return &member->data[index];
}

bool Activation_Result__rosidl_typesupport_introspection_c__resize_function__Resource__requested_resources(
  void * untyped_member, size_t size)
{
  dhtt_msgs__msg__Resource__Sequence * member =
    (dhtt_msgs__msg__Resource__Sequence *)(untyped_member);
  dhtt_msgs__msg__Resource__Sequence__fini(member);
  return dhtt_msgs__msg__Resource__Sequence__init(member, size);
}

size_t Activation_Result__rosidl_typesupport_introspection_c__size_function__Resource__owned_resources(
  const void * untyped_member)
{
  const dhtt_msgs__msg__Resource__Sequence * member =
    (const dhtt_msgs__msg__Resource__Sequence *)(untyped_member);
  return member->size;
}

const void * Activation_Result__rosidl_typesupport_introspection_c__get_const_function__Resource__owned_resources(
  const void * untyped_member, size_t index)
{
  const dhtt_msgs__msg__Resource__Sequence * member =
    (const dhtt_msgs__msg__Resource__Sequence *)(untyped_member);
  return &member->data[index];
}

void * Activation_Result__rosidl_typesupport_introspection_c__get_function__Resource__owned_resources(
  void * untyped_member, size_t index)
{
  dhtt_msgs__msg__Resource__Sequence * member =
    (dhtt_msgs__msg__Resource__Sequence *)(untyped_member);
  return &member->data[index];
}

bool Activation_Result__rosidl_typesupport_introspection_c__resize_function__Resource__owned_resources(
  void * untyped_member, size_t size)
{
  dhtt_msgs__msg__Resource__Sequence * member =
    (dhtt_msgs__msg__Resource__Sequence *)(untyped_member);
  dhtt_msgs__msg__Resource__Sequence__fini(member);
  return dhtt_msgs__msg__Resource__Sequence__init(member, size);
}

size_t Activation_Result__rosidl_typesupport_introspection_c__size_function__Resource__released_resources(
  const void * untyped_member)
{
  const dhtt_msgs__msg__Resource__Sequence * member =
    (const dhtt_msgs__msg__Resource__Sequence *)(untyped_member);
  return member->size;
}

const void * Activation_Result__rosidl_typesupport_introspection_c__get_const_function__Resource__released_resources(
  const void * untyped_member, size_t index)
{
  const dhtt_msgs__msg__Resource__Sequence * member =
    (const dhtt_msgs__msg__Resource__Sequence *)(untyped_member);
  return &member->data[index];
}

void * Activation_Result__rosidl_typesupport_introspection_c__get_function__Resource__released_resources(
  void * untyped_member, size_t index)
{
  dhtt_msgs__msg__Resource__Sequence * member =
    (dhtt_msgs__msg__Resource__Sequence *)(untyped_member);
  return &member->data[index];
}

bool Activation_Result__rosidl_typesupport_introspection_c__resize_function__Resource__released_resources(
  void * untyped_member, size_t size)
{
  dhtt_msgs__msg__Resource__Sequence * member =
    (dhtt_msgs__msg__Resource__Sequence *)(untyped_member);
  dhtt_msgs__msg__Resource__Sequence__fini(member);
  return dhtt_msgs__msg__Resource__Sequence__init(member, size);
}

size_t Activation_Result__rosidl_typesupport_introspection_c__size_function__Resource__passed_resources(
  const void * untyped_member)
{
  const dhtt_msgs__msg__Resource__Sequence * member =
    (const dhtt_msgs__msg__Resource__Sequence *)(untyped_member);
  return member->size;
}

const void * Activation_Result__rosidl_typesupport_introspection_c__get_const_function__Resource__passed_resources(
  const void * untyped_member, size_t index)
{
  const dhtt_msgs__msg__Resource__Sequence * member =
    (const dhtt_msgs__msg__Resource__Sequence *)(untyped_member);
  return &member->data[index];
}

void * Activation_Result__rosidl_typesupport_introspection_c__get_function__Resource__passed_resources(
  void * untyped_member, size_t index)
{
  dhtt_msgs__msg__Resource__Sequence * member =
    (dhtt_msgs__msg__Resource__Sequence *)(untyped_member);
  return &member->data[index];
}

bool Activation_Result__rosidl_typesupport_introspection_c__resize_function__Resource__passed_resources(
  void * untyped_member, size_t size)
{
  dhtt_msgs__msg__Resource__Sequence * member =
    (dhtt_msgs__msg__Resource__Sequence *)(untyped_member);
  dhtt_msgs__msg__Resource__Sequence__fini(member);
  return dhtt_msgs__msg__Resource__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember Activation_Result__rosidl_typesupport_introspection_c__Activation_Result_message_member_array[8] = {
  {
    "local_best_node",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__action__Activation_Result, local_best_node),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "requested_resources",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__action__Activation_Result, requested_resources),  // bytes offset in struct
    NULL,  // default value
    Activation_Result__rosidl_typesupport_introspection_c__size_function__Resource__requested_resources,  // size() function pointer
    Activation_Result__rosidl_typesupport_introspection_c__get_const_function__Resource__requested_resources,  // get_const(index) function pointer
    Activation_Result__rosidl_typesupport_introspection_c__get_function__Resource__requested_resources,  // get(index) function pointer
    Activation_Result__rosidl_typesupport_introspection_c__resize_function__Resource__requested_resources  // resize(index) function pointer
  },
  {
    "owned_resources",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__action__Activation_Result, owned_resources),  // bytes offset in struct
    NULL,  // default value
    Activation_Result__rosidl_typesupport_introspection_c__size_function__Resource__owned_resources,  // size() function pointer
    Activation_Result__rosidl_typesupport_introspection_c__get_const_function__Resource__owned_resources,  // get_const(index) function pointer
    Activation_Result__rosidl_typesupport_introspection_c__get_function__Resource__owned_resources,  // get(index) function pointer
    Activation_Result__rosidl_typesupport_introspection_c__resize_function__Resource__owned_resources  // resize(index) function pointer
  },
  {
    "done",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__action__Activation_Result, done),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "possible",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__action__Activation_Result, possible),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "activation_potential",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__action__Activation_Result, activation_potential),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "released_resources",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__action__Activation_Result, released_resources),  // bytes offset in struct
    NULL,  // default value
    Activation_Result__rosidl_typesupport_introspection_c__size_function__Resource__released_resources,  // size() function pointer
    Activation_Result__rosidl_typesupport_introspection_c__get_const_function__Resource__released_resources,  // get_const(index) function pointer
    Activation_Result__rosidl_typesupport_introspection_c__get_function__Resource__released_resources,  // get(index) function pointer
    Activation_Result__rosidl_typesupport_introspection_c__resize_function__Resource__released_resources  // resize(index) function pointer
  },
  {
    "passed_resources",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__action__Activation_Result, passed_resources),  // bytes offset in struct
    NULL,  // default value
    Activation_Result__rosidl_typesupport_introspection_c__size_function__Resource__passed_resources,  // size() function pointer
    Activation_Result__rosidl_typesupport_introspection_c__get_const_function__Resource__passed_resources,  // get_const(index) function pointer
    Activation_Result__rosidl_typesupport_introspection_c__get_function__Resource__passed_resources,  // get(index) function pointer
    Activation_Result__rosidl_typesupport_introspection_c__resize_function__Resource__passed_resources  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Activation_Result__rosidl_typesupport_introspection_c__Activation_Result_message_members = {
  "dhtt_msgs__action",  // message namespace
  "Activation_Result",  // message name
  8,  // number of fields
  sizeof(dhtt_msgs__action__Activation_Result),
  Activation_Result__rosidl_typesupport_introspection_c__Activation_Result_message_member_array,  // message members
  Activation_Result__rosidl_typesupport_introspection_c__Activation_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  Activation_Result__rosidl_typesupport_introspection_c__Activation_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Activation_Result__rosidl_typesupport_introspection_c__Activation_Result_message_type_support_handle = {
  0,
  &Activation_Result__rosidl_typesupport_introspection_c__Activation_Result_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dhtt_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, action, Activation_Result)() {
  Activation_Result__rosidl_typesupport_introspection_c__Activation_Result_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, msg, Resource)();
  Activation_Result__rosidl_typesupport_introspection_c__Activation_Result_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, msg, Resource)();
  Activation_Result__rosidl_typesupport_introspection_c__Activation_Result_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, msg, Resource)();
  Activation_Result__rosidl_typesupport_introspection_c__Activation_Result_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, msg, Resource)();
  if (!Activation_Result__rosidl_typesupport_introspection_c__Activation_Result_message_type_support_handle.typesupport_identifier) {
    Activation_Result__rosidl_typesupport_introspection_c__Activation_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Activation_Result__rosidl_typesupport_introspection_c__Activation_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "dhtt_msgs/action/detail/activation__rosidl_typesupport_introspection_c.h"
// already included above
// #include "dhtt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "dhtt_msgs/action/detail/activation__functions.h"
// already included above
// #include "dhtt_msgs/action/detail/activation__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void Activation_Feedback__rosidl_typesupport_introspection_c__Activation_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dhtt_msgs__action__Activation_Feedback__init(message_memory);
}

void Activation_Feedback__rosidl_typesupport_introspection_c__Activation_Feedback_fini_function(void * message_memory)
{
  dhtt_msgs__action__Activation_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Activation_Feedback__rosidl_typesupport_introspection_c__Activation_Feedback_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__action__Activation_Feedback, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Activation_Feedback__rosidl_typesupport_introspection_c__Activation_Feedback_message_members = {
  "dhtt_msgs__action",  // message namespace
  "Activation_Feedback",  // message name
  1,  // number of fields
  sizeof(dhtt_msgs__action__Activation_Feedback),
  Activation_Feedback__rosidl_typesupport_introspection_c__Activation_Feedback_message_member_array,  // message members
  Activation_Feedback__rosidl_typesupport_introspection_c__Activation_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  Activation_Feedback__rosidl_typesupport_introspection_c__Activation_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Activation_Feedback__rosidl_typesupport_introspection_c__Activation_Feedback_message_type_support_handle = {
  0,
  &Activation_Feedback__rosidl_typesupport_introspection_c__Activation_Feedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dhtt_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, action, Activation_Feedback)() {
  if (!Activation_Feedback__rosidl_typesupport_introspection_c__Activation_Feedback_message_type_support_handle.typesupport_identifier) {
    Activation_Feedback__rosidl_typesupport_introspection_c__Activation_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Activation_Feedback__rosidl_typesupport_introspection_c__Activation_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "dhtt_msgs/action/detail/activation__rosidl_typesupport_introspection_c.h"
// already included above
// #include "dhtt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "dhtt_msgs/action/detail/activation__functions.h"
// already included above
// #include "dhtt_msgs/action/detail/activation__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "dhtt_msgs/action/activation.h"
// Member `goal`
// already included above
// #include "dhtt_msgs/action/detail/activation__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Activation_SendGoal_Request__rosidl_typesupport_introspection_c__Activation_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dhtt_msgs__action__Activation_SendGoal_Request__init(message_memory);
}

void Activation_SendGoal_Request__rosidl_typesupport_introspection_c__Activation_SendGoal_Request_fini_function(void * message_memory)
{
  dhtt_msgs__action__Activation_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Activation_SendGoal_Request__rosidl_typesupport_introspection_c__Activation_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__action__Activation_SendGoal_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__action__Activation_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Activation_SendGoal_Request__rosidl_typesupport_introspection_c__Activation_SendGoal_Request_message_members = {
  "dhtt_msgs__action",  // message namespace
  "Activation_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(dhtt_msgs__action__Activation_SendGoal_Request),
  Activation_SendGoal_Request__rosidl_typesupport_introspection_c__Activation_SendGoal_Request_message_member_array,  // message members
  Activation_SendGoal_Request__rosidl_typesupport_introspection_c__Activation_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  Activation_SendGoal_Request__rosidl_typesupport_introspection_c__Activation_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Activation_SendGoal_Request__rosidl_typesupport_introspection_c__Activation_SendGoal_Request_message_type_support_handle = {
  0,
  &Activation_SendGoal_Request__rosidl_typesupport_introspection_c__Activation_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dhtt_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, action, Activation_SendGoal_Request)() {
  Activation_SendGoal_Request__rosidl_typesupport_introspection_c__Activation_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  Activation_SendGoal_Request__rosidl_typesupport_introspection_c__Activation_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, action, Activation_Goal)();
  if (!Activation_SendGoal_Request__rosidl_typesupport_introspection_c__Activation_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    Activation_SendGoal_Request__rosidl_typesupport_introspection_c__Activation_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Activation_SendGoal_Request__rosidl_typesupport_introspection_c__Activation_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "dhtt_msgs/action/detail/activation__rosidl_typesupport_introspection_c.h"
// already included above
// #include "dhtt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "dhtt_msgs/action/detail/activation__functions.h"
// already included above
// #include "dhtt_msgs/action/detail/activation__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Activation_SendGoal_Response__rosidl_typesupport_introspection_c__Activation_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dhtt_msgs__action__Activation_SendGoal_Response__init(message_memory);
}

void Activation_SendGoal_Response__rosidl_typesupport_introspection_c__Activation_SendGoal_Response_fini_function(void * message_memory)
{
  dhtt_msgs__action__Activation_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Activation_SendGoal_Response__rosidl_typesupport_introspection_c__Activation_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__action__Activation_SendGoal_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__action__Activation_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Activation_SendGoal_Response__rosidl_typesupport_introspection_c__Activation_SendGoal_Response_message_members = {
  "dhtt_msgs__action",  // message namespace
  "Activation_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(dhtt_msgs__action__Activation_SendGoal_Response),
  Activation_SendGoal_Response__rosidl_typesupport_introspection_c__Activation_SendGoal_Response_message_member_array,  // message members
  Activation_SendGoal_Response__rosidl_typesupport_introspection_c__Activation_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  Activation_SendGoal_Response__rosidl_typesupport_introspection_c__Activation_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Activation_SendGoal_Response__rosidl_typesupport_introspection_c__Activation_SendGoal_Response_message_type_support_handle = {
  0,
  &Activation_SendGoal_Response__rosidl_typesupport_introspection_c__Activation_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dhtt_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, action, Activation_SendGoal_Response)() {
  Activation_SendGoal_Response__rosidl_typesupport_introspection_c__Activation_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!Activation_SendGoal_Response__rosidl_typesupport_introspection_c__Activation_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    Activation_SendGoal_Response__rosidl_typesupport_introspection_c__Activation_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Activation_SendGoal_Response__rosidl_typesupport_introspection_c__Activation_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "dhtt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "dhtt_msgs/action/detail/activation__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers dhtt_msgs__action__detail__activation__rosidl_typesupport_introspection_c__Activation_SendGoal_service_members = {
  "dhtt_msgs__action",  // service namespace
  "Activation_SendGoal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // dhtt_msgs__action__detail__activation__rosidl_typesupport_introspection_c__Activation_SendGoal_Request_message_type_support_handle,
  NULL  // response message
  // dhtt_msgs__action__detail__activation__rosidl_typesupport_introspection_c__Activation_SendGoal_Response_message_type_support_handle
};

static rosidl_service_type_support_t dhtt_msgs__action__detail__activation__rosidl_typesupport_introspection_c__Activation_SendGoal_service_type_support_handle = {
  0,
  &dhtt_msgs__action__detail__activation__rosidl_typesupport_introspection_c__Activation_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, action, Activation_SendGoal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, action, Activation_SendGoal_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dhtt_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, action, Activation_SendGoal)() {
  if (!dhtt_msgs__action__detail__activation__rosidl_typesupport_introspection_c__Activation_SendGoal_service_type_support_handle.typesupport_identifier) {
    dhtt_msgs__action__detail__activation__rosidl_typesupport_introspection_c__Activation_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)dhtt_msgs__action__detail__activation__rosidl_typesupport_introspection_c__Activation_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, action, Activation_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, action, Activation_SendGoal_Response)()->data;
  }

  return &dhtt_msgs__action__detail__activation__rosidl_typesupport_introspection_c__Activation_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "dhtt_msgs/action/detail/activation__rosidl_typesupport_introspection_c.h"
// already included above
// #include "dhtt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "dhtt_msgs/action/detail/activation__functions.h"
// already included above
// #include "dhtt_msgs/action/detail/activation__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Activation_GetResult_Request__rosidl_typesupport_introspection_c__Activation_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dhtt_msgs__action__Activation_GetResult_Request__init(message_memory);
}

void Activation_GetResult_Request__rosidl_typesupport_introspection_c__Activation_GetResult_Request_fini_function(void * message_memory)
{
  dhtt_msgs__action__Activation_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Activation_GetResult_Request__rosidl_typesupport_introspection_c__Activation_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__action__Activation_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Activation_GetResult_Request__rosidl_typesupport_introspection_c__Activation_GetResult_Request_message_members = {
  "dhtt_msgs__action",  // message namespace
  "Activation_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(dhtt_msgs__action__Activation_GetResult_Request),
  Activation_GetResult_Request__rosidl_typesupport_introspection_c__Activation_GetResult_Request_message_member_array,  // message members
  Activation_GetResult_Request__rosidl_typesupport_introspection_c__Activation_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  Activation_GetResult_Request__rosidl_typesupport_introspection_c__Activation_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Activation_GetResult_Request__rosidl_typesupport_introspection_c__Activation_GetResult_Request_message_type_support_handle = {
  0,
  &Activation_GetResult_Request__rosidl_typesupport_introspection_c__Activation_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dhtt_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, action, Activation_GetResult_Request)() {
  Activation_GetResult_Request__rosidl_typesupport_introspection_c__Activation_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!Activation_GetResult_Request__rosidl_typesupport_introspection_c__Activation_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    Activation_GetResult_Request__rosidl_typesupport_introspection_c__Activation_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Activation_GetResult_Request__rosidl_typesupport_introspection_c__Activation_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "dhtt_msgs/action/detail/activation__rosidl_typesupport_introspection_c.h"
// already included above
// #include "dhtt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "dhtt_msgs/action/detail/activation__functions.h"
// already included above
// #include "dhtt_msgs/action/detail/activation__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "dhtt_msgs/action/activation.h"
// Member `result`
// already included above
// #include "dhtt_msgs/action/detail/activation__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Activation_GetResult_Response__rosidl_typesupport_introspection_c__Activation_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dhtt_msgs__action__Activation_GetResult_Response__init(message_memory);
}

void Activation_GetResult_Response__rosidl_typesupport_introspection_c__Activation_GetResult_Response_fini_function(void * message_memory)
{
  dhtt_msgs__action__Activation_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Activation_GetResult_Response__rosidl_typesupport_introspection_c__Activation_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__action__Activation_GetResult_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__action__Activation_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Activation_GetResult_Response__rosidl_typesupport_introspection_c__Activation_GetResult_Response_message_members = {
  "dhtt_msgs__action",  // message namespace
  "Activation_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(dhtt_msgs__action__Activation_GetResult_Response),
  Activation_GetResult_Response__rosidl_typesupport_introspection_c__Activation_GetResult_Response_message_member_array,  // message members
  Activation_GetResult_Response__rosidl_typesupport_introspection_c__Activation_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  Activation_GetResult_Response__rosidl_typesupport_introspection_c__Activation_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Activation_GetResult_Response__rosidl_typesupport_introspection_c__Activation_GetResult_Response_message_type_support_handle = {
  0,
  &Activation_GetResult_Response__rosidl_typesupport_introspection_c__Activation_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dhtt_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, action, Activation_GetResult_Response)() {
  Activation_GetResult_Response__rosidl_typesupport_introspection_c__Activation_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, action, Activation_Result)();
  if (!Activation_GetResult_Response__rosidl_typesupport_introspection_c__Activation_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    Activation_GetResult_Response__rosidl_typesupport_introspection_c__Activation_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Activation_GetResult_Response__rosidl_typesupport_introspection_c__Activation_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "dhtt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "dhtt_msgs/action/detail/activation__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers dhtt_msgs__action__detail__activation__rosidl_typesupport_introspection_c__Activation_GetResult_service_members = {
  "dhtt_msgs__action",  // service namespace
  "Activation_GetResult",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // dhtt_msgs__action__detail__activation__rosidl_typesupport_introspection_c__Activation_GetResult_Request_message_type_support_handle,
  NULL  // response message
  // dhtt_msgs__action__detail__activation__rosidl_typesupport_introspection_c__Activation_GetResult_Response_message_type_support_handle
};

static rosidl_service_type_support_t dhtt_msgs__action__detail__activation__rosidl_typesupport_introspection_c__Activation_GetResult_service_type_support_handle = {
  0,
  &dhtt_msgs__action__detail__activation__rosidl_typesupport_introspection_c__Activation_GetResult_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, action, Activation_GetResult_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, action, Activation_GetResult_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dhtt_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, action, Activation_GetResult)() {
  if (!dhtt_msgs__action__detail__activation__rosidl_typesupport_introspection_c__Activation_GetResult_service_type_support_handle.typesupport_identifier) {
    dhtt_msgs__action__detail__activation__rosidl_typesupport_introspection_c__Activation_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)dhtt_msgs__action__detail__activation__rosidl_typesupport_introspection_c__Activation_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, action, Activation_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, action, Activation_GetResult_Response)()->data;
  }

  return &dhtt_msgs__action__detail__activation__rosidl_typesupport_introspection_c__Activation_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "dhtt_msgs/action/detail/activation__rosidl_typesupport_introspection_c.h"
// already included above
// #include "dhtt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "dhtt_msgs/action/detail/activation__functions.h"
// already included above
// #include "dhtt_msgs/action/detail/activation__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "dhtt_msgs/action/activation.h"
// Member `feedback`
// already included above
// #include "dhtt_msgs/action/detail/activation__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Activation_FeedbackMessage__rosidl_typesupport_introspection_c__Activation_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dhtt_msgs__action__Activation_FeedbackMessage__init(message_memory);
}

void Activation_FeedbackMessage__rosidl_typesupport_introspection_c__Activation_FeedbackMessage_fini_function(void * message_memory)
{
  dhtt_msgs__action__Activation_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Activation_FeedbackMessage__rosidl_typesupport_introspection_c__Activation_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__action__Activation_FeedbackMessage, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feedback",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__action__Activation_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Activation_FeedbackMessage__rosidl_typesupport_introspection_c__Activation_FeedbackMessage_message_members = {
  "dhtt_msgs__action",  // message namespace
  "Activation_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(dhtt_msgs__action__Activation_FeedbackMessage),
  Activation_FeedbackMessage__rosidl_typesupport_introspection_c__Activation_FeedbackMessage_message_member_array,  // message members
  Activation_FeedbackMessage__rosidl_typesupport_introspection_c__Activation_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  Activation_FeedbackMessage__rosidl_typesupport_introspection_c__Activation_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Activation_FeedbackMessage__rosidl_typesupport_introspection_c__Activation_FeedbackMessage_message_type_support_handle = {
  0,
  &Activation_FeedbackMessage__rosidl_typesupport_introspection_c__Activation_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dhtt_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, action, Activation_FeedbackMessage)() {
  Activation_FeedbackMessage__rosidl_typesupport_introspection_c__Activation_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  Activation_FeedbackMessage__rosidl_typesupport_introspection_c__Activation_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, action, Activation_Feedback)();
  if (!Activation_FeedbackMessage__rosidl_typesupport_introspection_c__Activation_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    Activation_FeedbackMessage__rosidl_typesupport_introspection_c__Activation_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Activation_FeedbackMessage__rosidl_typesupport_introspection_c__Activation_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
