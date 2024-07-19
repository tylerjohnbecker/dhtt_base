// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from dhtt_msgs:srv/FetchInfo.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "dhtt_msgs/srv/detail/fetch_info__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace dhtt_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void FetchInfo_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dhtt_msgs::srv::FetchInfo_Request(_init);
}

void FetchInfo_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dhtt_msgs::srv::FetchInfo_Request *>(message_memory);
  typed_message->~FetchInfo_Request();
}

size_t size_function__FetchInfo_Request__keys(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__FetchInfo_Request__keys(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__FetchInfo_Request__keys(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void resize_function__FetchInfo_Request__keys(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember FetchInfo_Request_message_member_array[1] = {
  {
    "keys",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::srv::FetchInfo_Request, keys),  // bytes offset in struct
    nullptr,  // default value
    size_function__FetchInfo_Request__keys,  // size() function pointer
    get_const_function__FetchInfo_Request__keys,  // get_const(index) function pointer
    get_function__FetchInfo_Request__keys,  // get(index) function pointer
    resize_function__FetchInfo_Request__keys  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers FetchInfo_Request_message_members = {
  "dhtt_msgs::srv",  // message namespace
  "FetchInfo_Request",  // message name
  1,  // number of fields
  sizeof(dhtt_msgs::srv::FetchInfo_Request),
  FetchInfo_Request_message_member_array,  // message members
  FetchInfo_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  FetchInfo_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t FetchInfo_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &FetchInfo_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace dhtt_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dhtt_msgs::srv::FetchInfo_Request>()
{
  return &::dhtt_msgs::srv::rosidl_typesupport_introspection_cpp::FetchInfo_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dhtt_msgs, srv, FetchInfo_Request)() {
  return &::dhtt_msgs::srv::rosidl_typesupport_introspection_cpp::FetchInfo_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "dhtt_msgs/srv/detail/fetch_info__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace dhtt_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void FetchInfo_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dhtt_msgs::srv::FetchInfo_Response(_init);
}

void FetchInfo_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dhtt_msgs::srv::FetchInfo_Response *>(message_memory);
  typed_message->~FetchInfo_Response();
}

size_t size_function__FetchInfo_Response__information_pairs(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<dhtt_msgs::msg::Pair> *>(untyped_member);
  return member->size();
}

const void * get_const_function__FetchInfo_Response__information_pairs(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<dhtt_msgs::msg::Pair> *>(untyped_member);
  return &member[index];
}

void * get_function__FetchInfo_Response__information_pairs(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<dhtt_msgs::msg::Pair> *>(untyped_member);
  return &member[index];
}

void resize_function__FetchInfo_Response__information_pairs(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<dhtt_msgs::msg::Pair> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember FetchInfo_Response_message_member_array[1] = {
  {
    "information_pairs",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dhtt_msgs::msg::Pair>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::srv::FetchInfo_Response, information_pairs),  // bytes offset in struct
    nullptr,  // default value
    size_function__FetchInfo_Response__information_pairs,  // size() function pointer
    get_const_function__FetchInfo_Response__information_pairs,  // get_const(index) function pointer
    get_function__FetchInfo_Response__information_pairs,  // get(index) function pointer
    resize_function__FetchInfo_Response__information_pairs  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers FetchInfo_Response_message_members = {
  "dhtt_msgs::srv",  // message namespace
  "FetchInfo_Response",  // message name
  1,  // number of fields
  sizeof(dhtt_msgs::srv::FetchInfo_Response),
  FetchInfo_Response_message_member_array,  // message members
  FetchInfo_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  FetchInfo_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t FetchInfo_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &FetchInfo_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace dhtt_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dhtt_msgs::srv::FetchInfo_Response>()
{
  return &::dhtt_msgs::srv::rosidl_typesupport_introspection_cpp::FetchInfo_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dhtt_msgs, srv, FetchInfo_Response)() {
  return &::dhtt_msgs::srv::rosidl_typesupport_introspection_cpp::FetchInfo_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "dhtt_msgs/srv/detail/fetch_info__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace dhtt_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers FetchInfo_service_members = {
  "dhtt_msgs::srv",  // service namespace
  "FetchInfo",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<dhtt_msgs::srv::FetchInfo>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t FetchInfo_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &FetchInfo_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace dhtt_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<dhtt_msgs::srv::FetchInfo>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::dhtt_msgs::srv::rosidl_typesupport_introspection_cpp::FetchInfo_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::dhtt_msgs::srv::FetchInfo_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::dhtt_msgs::srv::FetchInfo_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dhtt_msgs, srv, FetchInfo)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<dhtt_msgs::srv::FetchInfo>();
}

#ifdef __cplusplus
}
#endif
