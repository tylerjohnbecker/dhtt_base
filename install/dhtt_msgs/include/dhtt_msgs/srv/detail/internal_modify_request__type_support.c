// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dhtt_msgs:srv/InternalModifyRequest.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dhtt_msgs/srv/detail/internal_modify_request__rosidl_typesupport_introspection_c.h"
#include "dhtt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dhtt_msgs/srv/detail/internal_modify_request__functions.h"
#include "dhtt_msgs/srv/detail/internal_modify_request__struct.h"


// Include directives for member types
// Member `node_name`
// Member `plugin_name`
// Member `params`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void InternalModifyRequest_Request__rosidl_typesupport_introspection_c__InternalModifyRequest_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dhtt_msgs__srv__InternalModifyRequest_Request__init(message_memory);
}

void InternalModifyRequest_Request__rosidl_typesupport_introspection_c__InternalModifyRequest_Request_fini_function(void * message_memory)
{
  dhtt_msgs__srv__InternalModifyRequest_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember InternalModifyRequest_Request__rosidl_typesupport_introspection_c__InternalModifyRequest_Request_message_member_array[4] = {
  {
    "type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__srv__InternalModifyRequest_Request, type),  // bytes offset in struct
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
    offsetof(dhtt_msgs__srv__InternalModifyRequest_Request, node_name),  // bytes offset in struct
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
    offsetof(dhtt_msgs__srv__InternalModifyRequest_Request, plugin_name),  // bytes offset in struct
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
    offsetof(dhtt_msgs__srv__InternalModifyRequest_Request, params),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers InternalModifyRequest_Request__rosidl_typesupport_introspection_c__InternalModifyRequest_Request_message_members = {
  "dhtt_msgs__srv",  // message namespace
  "InternalModifyRequest_Request",  // message name
  4,  // number of fields
  sizeof(dhtt_msgs__srv__InternalModifyRequest_Request),
  InternalModifyRequest_Request__rosidl_typesupport_introspection_c__InternalModifyRequest_Request_message_member_array,  // message members
  InternalModifyRequest_Request__rosidl_typesupport_introspection_c__InternalModifyRequest_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  InternalModifyRequest_Request__rosidl_typesupport_introspection_c__InternalModifyRequest_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t InternalModifyRequest_Request__rosidl_typesupport_introspection_c__InternalModifyRequest_Request_message_type_support_handle = {
  0,
  &InternalModifyRequest_Request__rosidl_typesupport_introspection_c__InternalModifyRequest_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dhtt_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, srv, InternalModifyRequest_Request)() {
  if (!InternalModifyRequest_Request__rosidl_typesupport_introspection_c__InternalModifyRequest_Request_message_type_support_handle.typesupport_identifier) {
    InternalModifyRequest_Request__rosidl_typesupport_introspection_c__InternalModifyRequest_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &InternalModifyRequest_Request__rosidl_typesupport_introspection_c__InternalModifyRequest_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "dhtt_msgs/srv/detail/internal_modify_request__rosidl_typesupport_introspection_c.h"
// already included above
// #include "dhtt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "dhtt_msgs/srv/detail/internal_modify_request__functions.h"
// already included above
// #include "dhtt_msgs/srv/detail/internal_modify_request__struct.h"


// Include directives for member types
// Member `error_msg`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void InternalModifyRequest_Response__rosidl_typesupport_introspection_c__InternalModifyRequest_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dhtt_msgs__srv__InternalModifyRequest_Response__init(message_memory);
}

void InternalModifyRequest_Response__rosidl_typesupport_introspection_c__InternalModifyRequest_Response_fini_function(void * message_memory)
{
  dhtt_msgs__srv__InternalModifyRequest_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember InternalModifyRequest_Response__rosidl_typesupport_introspection_c__InternalModifyRequest_Response_message_member_array[2] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__srv__InternalModifyRequest_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_msg",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs__srv__InternalModifyRequest_Response, error_msg),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers InternalModifyRequest_Response__rosidl_typesupport_introspection_c__InternalModifyRequest_Response_message_members = {
  "dhtt_msgs__srv",  // message namespace
  "InternalModifyRequest_Response",  // message name
  2,  // number of fields
  sizeof(dhtt_msgs__srv__InternalModifyRequest_Response),
  InternalModifyRequest_Response__rosidl_typesupport_introspection_c__InternalModifyRequest_Response_message_member_array,  // message members
  InternalModifyRequest_Response__rosidl_typesupport_introspection_c__InternalModifyRequest_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  InternalModifyRequest_Response__rosidl_typesupport_introspection_c__InternalModifyRequest_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t InternalModifyRequest_Response__rosidl_typesupport_introspection_c__InternalModifyRequest_Response_message_type_support_handle = {
  0,
  &InternalModifyRequest_Response__rosidl_typesupport_introspection_c__InternalModifyRequest_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dhtt_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, srv, InternalModifyRequest_Response)() {
  if (!InternalModifyRequest_Response__rosidl_typesupport_introspection_c__InternalModifyRequest_Response_message_type_support_handle.typesupport_identifier) {
    InternalModifyRequest_Response__rosidl_typesupport_introspection_c__InternalModifyRequest_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &InternalModifyRequest_Response__rosidl_typesupport_introspection_c__InternalModifyRequest_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "dhtt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "dhtt_msgs/srv/detail/internal_modify_request__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers dhtt_msgs__srv__detail__internal_modify_request__rosidl_typesupport_introspection_c__InternalModifyRequest_service_members = {
  "dhtt_msgs__srv",  // service namespace
  "InternalModifyRequest",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // dhtt_msgs__srv__detail__internal_modify_request__rosidl_typesupport_introspection_c__InternalModifyRequest_Request_message_type_support_handle,
  NULL  // response message
  // dhtt_msgs__srv__detail__internal_modify_request__rosidl_typesupport_introspection_c__InternalModifyRequest_Response_message_type_support_handle
};

static rosidl_service_type_support_t dhtt_msgs__srv__detail__internal_modify_request__rosidl_typesupport_introspection_c__InternalModifyRequest_service_type_support_handle = {
  0,
  &dhtt_msgs__srv__detail__internal_modify_request__rosidl_typesupport_introspection_c__InternalModifyRequest_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, srv, InternalModifyRequest_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, srv, InternalModifyRequest_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dhtt_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, srv, InternalModifyRequest)() {
  if (!dhtt_msgs__srv__detail__internal_modify_request__rosidl_typesupport_introspection_c__InternalModifyRequest_service_type_support_handle.typesupport_identifier) {
    dhtt_msgs__srv__detail__internal_modify_request__rosidl_typesupport_introspection_c__InternalModifyRequest_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)dhtt_msgs__srv__detail__internal_modify_request__rosidl_typesupport_introspection_c__InternalModifyRequest_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, srv, InternalModifyRequest_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dhtt_msgs, srv, InternalModifyRequest_Response)()->data;
  }

  return &dhtt_msgs__srv__detail__internal_modify_request__rosidl_typesupport_introspection_c__InternalModifyRequest_service_type_support_handle;
}
