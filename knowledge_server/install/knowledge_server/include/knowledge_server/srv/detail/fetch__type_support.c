// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from knowledge_server:srv/Fetch.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "knowledge_server/srv/detail/fetch__rosidl_typesupport_introspection_c.h"
#include "knowledge_server/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "knowledge_server/srv/detail/fetch__functions.h"
#include "knowledge_server/srv/detail/fetch__struct.h"


// Include directives for member types
// Member `keys`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Fetch_Request__rosidl_typesupport_introspection_c__Fetch_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  knowledge_server__srv__Fetch_Request__init(message_memory);
}

void Fetch_Request__rosidl_typesupport_introspection_c__Fetch_Request_fini_function(void * message_memory)
{
  knowledge_server__srv__Fetch_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Fetch_Request__rosidl_typesupport_introspection_c__Fetch_Request_message_member_array[1] = {
  {
    "keys",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(knowledge_server__srv__Fetch_Request, keys),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Fetch_Request__rosidl_typesupport_introspection_c__Fetch_Request_message_members = {
  "knowledge_server__srv",  // message namespace
  "Fetch_Request",  // message name
  1,  // number of fields
  sizeof(knowledge_server__srv__Fetch_Request),
  Fetch_Request__rosidl_typesupport_introspection_c__Fetch_Request_message_member_array,  // message members
  Fetch_Request__rosidl_typesupport_introspection_c__Fetch_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  Fetch_Request__rosidl_typesupport_introspection_c__Fetch_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Fetch_Request__rosidl_typesupport_introspection_c__Fetch_Request_message_type_support_handle = {
  0,
  &Fetch_Request__rosidl_typesupport_introspection_c__Fetch_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_knowledge_server
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, knowledge_server, srv, Fetch_Request)() {
  if (!Fetch_Request__rosidl_typesupport_introspection_c__Fetch_Request_message_type_support_handle.typesupport_identifier) {
    Fetch_Request__rosidl_typesupport_introspection_c__Fetch_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Fetch_Request__rosidl_typesupport_introspection_c__Fetch_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "knowledge_server/srv/detail/fetch__rosidl_typesupport_introspection_c.h"
// already included above
// #include "knowledge_server/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "knowledge_server/srv/detail/fetch__functions.h"
// already included above
// #include "knowledge_server/srv/detail/fetch__struct.h"


// Include directives for member types
// Member `pairs`
#include "knowledge_server/msg/pair.h"
// Member `pairs`
#include "knowledge_server/msg/detail/pair__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Fetch_Response__rosidl_typesupport_introspection_c__Fetch_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  knowledge_server__srv__Fetch_Response__init(message_memory);
}

void Fetch_Response__rosidl_typesupport_introspection_c__Fetch_Response_fini_function(void * message_memory)
{
  knowledge_server__srv__Fetch_Response__fini(message_memory);
}

size_t Fetch_Response__rosidl_typesupport_introspection_c__size_function__Pair__pairs(
  const void * untyped_member)
{
  const knowledge_server__msg__Pair__Sequence * member =
    (const knowledge_server__msg__Pair__Sequence *)(untyped_member);
  return member->size;
}

const void * Fetch_Response__rosidl_typesupport_introspection_c__get_const_function__Pair__pairs(
  const void * untyped_member, size_t index)
{
  const knowledge_server__msg__Pair__Sequence * member =
    (const knowledge_server__msg__Pair__Sequence *)(untyped_member);
  return &member->data[index];
}

void * Fetch_Response__rosidl_typesupport_introspection_c__get_function__Pair__pairs(
  void * untyped_member, size_t index)
{
  knowledge_server__msg__Pair__Sequence * member =
    (knowledge_server__msg__Pair__Sequence *)(untyped_member);
  return &member->data[index];
}

bool Fetch_Response__rosidl_typesupport_introspection_c__resize_function__Pair__pairs(
  void * untyped_member, size_t size)
{
  knowledge_server__msg__Pair__Sequence * member =
    (knowledge_server__msg__Pair__Sequence *)(untyped_member);
  knowledge_server__msg__Pair__Sequence__fini(member);
  return knowledge_server__msg__Pair__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember Fetch_Response__rosidl_typesupport_introspection_c__Fetch_Response_message_member_array[1] = {
  {
    "pairs",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(knowledge_server__srv__Fetch_Response, pairs),  // bytes offset in struct
    NULL,  // default value
    Fetch_Response__rosidl_typesupport_introspection_c__size_function__Pair__pairs,  // size() function pointer
    Fetch_Response__rosidl_typesupport_introspection_c__get_const_function__Pair__pairs,  // get_const(index) function pointer
    Fetch_Response__rosidl_typesupport_introspection_c__get_function__Pair__pairs,  // get(index) function pointer
    Fetch_Response__rosidl_typesupport_introspection_c__resize_function__Pair__pairs  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Fetch_Response__rosidl_typesupport_introspection_c__Fetch_Response_message_members = {
  "knowledge_server__srv",  // message namespace
  "Fetch_Response",  // message name
  1,  // number of fields
  sizeof(knowledge_server__srv__Fetch_Response),
  Fetch_Response__rosidl_typesupport_introspection_c__Fetch_Response_message_member_array,  // message members
  Fetch_Response__rosidl_typesupport_introspection_c__Fetch_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  Fetch_Response__rosidl_typesupport_introspection_c__Fetch_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Fetch_Response__rosidl_typesupport_introspection_c__Fetch_Response_message_type_support_handle = {
  0,
  &Fetch_Response__rosidl_typesupport_introspection_c__Fetch_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_knowledge_server
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, knowledge_server, srv, Fetch_Response)() {
  Fetch_Response__rosidl_typesupport_introspection_c__Fetch_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, knowledge_server, msg, Pair)();
  if (!Fetch_Response__rosidl_typesupport_introspection_c__Fetch_Response_message_type_support_handle.typesupport_identifier) {
    Fetch_Response__rosidl_typesupport_introspection_c__Fetch_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Fetch_Response__rosidl_typesupport_introspection_c__Fetch_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "knowledge_server/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "knowledge_server/srv/detail/fetch__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers knowledge_server__srv__detail__fetch__rosidl_typesupport_introspection_c__Fetch_service_members = {
  "knowledge_server__srv",  // service namespace
  "Fetch",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // knowledge_server__srv__detail__fetch__rosidl_typesupport_introspection_c__Fetch_Request_message_type_support_handle,
  NULL  // response message
  // knowledge_server__srv__detail__fetch__rosidl_typesupport_introspection_c__Fetch_Response_message_type_support_handle
};

static rosidl_service_type_support_t knowledge_server__srv__detail__fetch__rosidl_typesupport_introspection_c__Fetch_service_type_support_handle = {
  0,
  &knowledge_server__srv__detail__fetch__rosidl_typesupport_introspection_c__Fetch_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, knowledge_server, srv, Fetch_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, knowledge_server, srv, Fetch_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_knowledge_server
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, knowledge_server, srv, Fetch)() {
  if (!knowledge_server__srv__detail__fetch__rosidl_typesupport_introspection_c__Fetch_service_type_support_handle.typesupport_identifier) {
    knowledge_server__srv__detail__fetch__rosidl_typesupport_introspection_c__Fetch_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)knowledge_server__srv__detail__fetch__rosidl_typesupport_introspection_c__Fetch_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, knowledge_server, srv, Fetch_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, knowledge_server, srv, Fetch_Response)()->data;
  }

  return &knowledge_server__srv__detail__fetch__rosidl_typesupport_introspection_c__Fetch_service_type_support_handle;
}
