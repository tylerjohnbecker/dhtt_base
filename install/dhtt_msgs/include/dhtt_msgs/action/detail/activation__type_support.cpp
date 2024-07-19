// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from dhtt_msgs:action/Activation.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "dhtt_msgs/action/detail/activation__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace dhtt_msgs
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void Activation_Goal_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dhtt_msgs::action::Activation_Goal(_init);
}

void Activation_Goal_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dhtt_msgs::action::Activation_Goal *>(message_memory);
  typed_message->~Activation_Goal();
}

size_t size_function__Activation_Goal__passed_resources(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<dhtt_msgs::msg::Resource> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Activation_Goal__passed_resources(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<dhtt_msgs::msg::Resource> *>(untyped_member);
  return &member[index];
}

void * get_function__Activation_Goal__passed_resources(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<dhtt_msgs::msg::Resource> *>(untyped_member);
  return &member[index];
}

void resize_function__Activation_Goal__passed_resources(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<dhtt_msgs::msg::Resource> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Activation_Goal__granted_resources(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<dhtt_msgs::msg::Resource> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Activation_Goal__granted_resources(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<dhtt_msgs::msg::Resource> *>(untyped_member);
  return &member[index];
}

void * get_function__Activation_Goal__granted_resources(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<dhtt_msgs::msg::Resource> *>(untyped_member);
  return &member[index];
}

void resize_function__Activation_Goal__granted_resources(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<dhtt_msgs::msg::Resource> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Activation_Goal_message_member_array[3] = {
  {
    "passed_resources",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dhtt_msgs::msg::Resource>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::action::Activation_Goal, passed_resources),  // bytes offset in struct
    nullptr,  // default value
    size_function__Activation_Goal__passed_resources,  // size() function pointer
    get_const_function__Activation_Goal__passed_resources,  // get_const(index) function pointer
    get_function__Activation_Goal__passed_resources,  // get(index) function pointer
    resize_function__Activation_Goal__passed_resources  // resize(index) function pointer
  },
  {
    "granted_resources",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dhtt_msgs::msg::Resource>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::action::Activation_Goal, granted_resources),  // bytes offset in struct
    nullptr,  // default value
    size_function__Activation_Goal__granted_resources,  // size() function pointer
    get_const_function__Activation_Goal__granted_resources,  // get_const(index) function pointer
    get_function__Activation_Goal__granted_resources,  // get(index) function pointer
    resize_function__Activation_Goal__granted_resources  // resize(index) function pointer
  },
  {
    "success",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::action::Activation_Goal, success),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Activation_Goal_message_members = {
  "dhtt_msgs::action",  // message namespace
  "Activation_Goal",  // message name
  3,  // number of fields
  sizeof(dhtt_msgs::action::Activation_Goal),
  Activation_Goal_message_member_array,  // message members
  Activation_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  Activation_Goal_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Activation_Goal_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Activation_Goal_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace dhtt_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dhtt_msgs::action::Activation_Goal>()
{
  return &::dhtt_msgs::action::rosidl_typesupport_introspection_cpp::Activation_Goal_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dhtt_msgs, action, Activation_Goal)() {
  return &::dhtt_msgs::action::rosidl_typesupport_introspection_cpp::Activation_Goal_message_type_support_handle;
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
// #include "dhtt_msgs/action/detail/activation__struct.hpp"
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

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void Activation_Result_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dhtt_msgs::action::Activation_Result(_init);
}

void Activation_Result_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dhtt_msgs::action::Activation_Result *>(message_memory);
  typed_message->~Activation_Result();
}

size_t size_function__Activation_Result__requested_resources(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<dhtt_msgs::msg::Resource> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Activation_Result__requested_resources(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<dhtt_msgs::msg::Resource> *>(untyped_member);
  return &member[index];
}

void * get_function__Activation_Result__requested_resources(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<dhtt_msgs::msg::Resource> *>(untyped_member);
  return &member[index];
}

void resize_function__Activation_Result__requested_resources(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<dhtt_msgs::msg::Resource> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Activation_Result__owned_resources(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<dhtt_msgs::msg::Resource> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Activation_Result__owned_resources(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<dhtt_msgs::msg::Resource> *>(untyped_member);
  return &member[index];
}

void * get_function__Activation_Result__owned_resources(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<dhtt_msgs::msg::Resource> *>(untyped_member);
  return &member[index];
}

void resize_function__Activation_Result__owned_resources(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<dhtt_msgs::msg::Resource> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Activation_Result__released_resources(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<dhtt_msgs::msg::Resource> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Activation_Result__released_resources(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<dhtt_msgs::msg::Resource> *>(untyped_member);
  return &member[index];
}

void * get_function__Activation_Result__released_resources(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<dhtt_msgs::msg::Resource> *>(untyped_member);
  return &member[index];
}

void resize_function__Activation_Result__released_resources(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<dhtt_msgs::msg::Resource> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Activation_Result__passed_resources(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<dhtt_msgs::msg::Resource> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Activation_Result__passed_resources(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<dhtt_msgs::msg::Resource> *>(untyped_member);
  return &member[index];
}

void * get_function__Activation_Result__passed_resources(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<dhtt_msgs::msg::Resource> *>(untyped_member);
  return &member[index];
}

void resize_function__Activation_Result__passed_resources(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<dhtt_msgs::msg::Resource> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Activation_Result_message_member_array[8] = {
  {
    "local_best_node",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::action::Activation_Result, local_best_node),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "requested_resources",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dhtt_msgs::msg::Resource>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::action::Activation_Result, requested_resources),  // bytes offset in struct
    nullptr,  // default value
    size_function__Activation_Result__requested_resources,  // size() function pointer
    get_const_function__Activation_Result__requested_resources,  // get_const(index) function pointer
    get_function__Activation_Result__requested_resources,  // get(index) function pointer
    resize_function__Activation_Result__requested_resources  // resize(index) function pointer
  },
  {
    "owned_resources",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dhtt_msgs::msg::Resource>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::action::Activation_Result, owned_resources),  // bytes offset in struct
    nullptr,  // default value
    size_function__Activation_Result__owned_resources,  // size() function pointer
    get_const_function__Activation_Result__owned_resources,  // get_const(index) function pointer
    get_function__Activation_Result__owned_resources,  // get(index) function pointer
    resize_function__Activation_Result__owned_resources  // resize(index) function pointer
  },
  {
    "done",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::action::Activation_Result, done),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "possible",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::action::Activation_Result, possible),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "activation_potential",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::action::Activation_Result, activation_potential),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "released_resources",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dhtt_msgs::msg::Resource>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::action::Activation_Result, released_resources),  // bytes offset in struct
    nullptr,  // default value
    size_function__Activation_Result__released_resources,  // size() function pointer
    get_const_function__Activation_Result__released_resources,  // get_const(index) function pointer
    get_function__Activation_Result__released_resources,  // get(index) function pointer
    resize_function__Activation_Result__released_resources  // resize(index) function pointer
  },
  {
    "passed_resources",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dhtt_msgs::msg::Resource>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::action::Activation_Result, passed_resources),  // bytes offset in struct
    nullptr,  // default value
    size_function__Activation_Result__passed_resources,  // size() function pointer
    get_const_function__Activation_Result__passed_resources,  // get_const(index) function pointer
    get_function__Activation_Result__passed_resources,  // get(index) function pointer
    resize_function__Activation_Result__passed_resources  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Activation_Result_message_members = {
  "dhtt_msgs::action",  // message namespace
  "Activation_Result",  // message name
  8,  // number of fields
  sizeof(dhtt_msgs::action::Activation_Result),
  Activation_Result_message_member_array,  // message members
  Activation_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  Activation_Result_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Activation_Result_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Activation_Result_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace dhtt_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dhtt_msgs::action::Activation_Result>()
{
  return &::dhtt_msgs::action::rosidl_typesupport_introspection_cpp::Activation_Result_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dhtt_msgs, action, Activation_Result)() {
  return &::dhtt_msgs::action::rosidl_typesupport_introspection_cpp::Activation_Result_message_type_support_handle;
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
// #include "dhtt_msgs/action/detail/activation__struct.hpp"
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

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void Activation_Feedback_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dhtt_msgs::action::Activation_Feedback(_init);
}

void Activation_Feedback_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dhtt_msgs::action::Activation_Feedback *>(message_memory);
  typed_message->~Activation_Feedback();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Activation_Feedback_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::action::Activation_Feedback, structure_needs_at_least_one_member),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Activation_Feedback_message_members = {
  "dhtt_msgs::action",  // message namespace
  "Activation_Feedback",  // message name
  1,  // number of fields
  sizeof(dhtt_msgs::action::Activation_Feedback),
  Activation_Feedback_message_member_array,  // message members
  Activation_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  Activation_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Activation_Feedback_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Activation_Feedback_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace dhtt_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dhtt_msgs::action::Activation_Feedback>()
{
  return &::dhtt_msgs::action::rosidl_typesupport_introspection_cpp::Activation_Feedback_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dhtt_msgs, action, Activation_Feedback)() {
  return &::dhtt_msgs::action::rosidl_typesupport_introspection_cpp::Activation_Feedback_message_type_support_handle;
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
// #include "dhtt_msgs/action/detail/activation__struct.hpp"
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

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void Activation_SendGoal_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dhtt_msgs::action::Activation_SendGoal_Request(_init);
}

void Activation_SendGoal_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dhtt_msgs::action::Activation_SendGoal_Request *>(message_memory);
  typed_message->~Activation_SendGoal_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Activation_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<unique_identifier_msgs::msg::UUID>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::action::Activation_SendGoal_Request, goal_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "goal",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dhtt_msgs::action::Activation_Goal>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::action::Activation_SendGoal_Request, goal),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Activation_SendGoal_Request_message_members = {
  "dhtt_msgs::action",  // message namespace
  "Activation_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(dhtt_msgs::action::Activation_SendGoal_Request),
  Activation_SendGoal_Request_message_member_array,  // message members
  Activation_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  Activation_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Activation_SendGoal_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Activation_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace dhtt_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dhtt_msgs::action::Activation_SendGoal_Request>()
{
  return &::dhtt_msgs::action::rosidl_typesupport_introspection_cpp::Activation_SendGoal_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dhtt_msgs, action, Activation_SendGoal_Request)() {
  return &::dhtt_msgs::action::rosidl_typesupport_introspection_cpp::Activation_SendGoal_Request_message_type_support_handle;
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
// #include "dhtt_msgs/action/detail/activation__struct.hpp"
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

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void Activation_SendGoal_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dhtt_msgs::action::Activation_SendGoal_Response(_init);
}

void Activation_SendGoal_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dhtt_msgs::action::Activation_SendGoal_Response *>(message_memory);
  typed_message->~Activation_SendGoal_Response();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Activation_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::action::Activation_SendGoal_Response, accepted),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "stamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Time>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::action::Activation_SendGoal_Response, stamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Activation_SendGoal_Response_message_members = {
  "dhtt_msgs::action",  // message namespace
  "Activation_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(dhtt_msgs::action::Activation_SendGoal_Response),
  Activation_SendGoal_Response_message_member_array,  // message members
  Activation_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  Activation_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Activation_SendGoal_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Activation_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace dhtt_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dhtt_msgs::action::Activation_SendGoal_Response>()
{
  return &::dhtt_msgs::action::rosidl_typesupport_introspection_cpp::Activation_SendGoal_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dhtt_msgs, action, Activation_SendGoal_Response)() {
  return &::dhtt_msgs::action::rosidl_typesupport_introspection_cpp::Activation_SendGoal_Response_message_type_support_handle;
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
// #include "dhtt_msgs/action/detail/activation__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace dhtt_msgs
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers Activation_SendGoal_service_members = {
  "dhtt_msgs::action",  // service namespace
  "Activation_SendGoal",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<dhtt_msgs::action::Activation_SendGoal>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t Activation_SendGoal_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Activation_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace dhtt_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<dhtt_msgs::action::Activation_SendGoal>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::dhtt_msgs::action::rosidl_typesupport_introspection_cpp::Activation_SendGoal_service_type_support_handle;
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
        ::dhtt_msgs::action::Activation_SendGoal_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::dhtt_msgs::action::Activation_SendGoal_Response
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
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dhtt_msgs, action, Activation_SendGoal)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<dhtt_msgs::action::Activation_SendGoal>();
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
// #include "dhtt_msgs/action/detail/activation__struct.hpp"
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

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void Activation_GetResult_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dhtt_msgs::action::Activation_GetResult_Request(_init);
}

void Activation_GetResult_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dhtt_msgs::action::Activation_GetResult_Request *>(message_memory);
  typed_message->~Activation_GetResult_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Activation_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<unique_identifier_msgs::msg::UUID>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::action::Activation_GetResult_Request, goal_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Activation_GetResult_Request_message_members = {
  "dhtt_msgs::action",  // message namespace
  "Activation_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(dhtt_msgs::action::Activation_GetResult_Request),
  Activation_GetResult_Request_message_member_array,  // message members
  Activation_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  Activation_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Activation_GetResult_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Activation_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace dhtt_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dhtt_msgs::action::Activation_GetResult_Request>()
{
  return &::dhtt_msgs::action::rosidl_typesupport_introspection_cpp::Activation_GetResult_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dhtt_msgs, action, Activation_GetResult_Request)() {
  return &::dhtt_msgs::action::rosidl_typesupport_introspection_cpp::Activation_GetResult_Request_message_type_support_handle;
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
// #include "dhtt_msgs/action/detail/activation__struct.hpp"
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

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void Activation_GetResult_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dhtt_msgs::action::Activation_GetResult_Response(_init);
}

void Activation_GetResult_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dhtt_msgs::action::Activation_GetResult_Response *>(message_memory);
  typed_message->~Activation_GetResult_Response();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Activation_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::action::Activation_GetResult_Response, status),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "result",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dhtt_msgs::action::Activation_Result>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::action::Activation_GetResult_Response, result),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Activation_GetResult_Response_message_members = {
  "dhtt_msgs::action",  // message namespace
  "Activation_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(dhtt_msgs::action::Activation_GetResult_Response),
  Activation_GetResult_Response_message_member_array,  // message members
  Activation_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  Activation_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Activation_GetResult_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Activation_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace dhtt_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dhtt_msgs::action::Activation_GetResult_Response>()
{
  return &::dhtt_msgs::action::rosidl_typesupport_introspection_cpp::Activation_GetResult_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dhtt_msgs, action, Activation_GetResult_Response)() {
  return &::dhtt_msgs::action::rosidl_typesupport_introspection_cpp::Activation_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "dhtt_msgs/action/detail/activation__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace dhtt_msgs
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers Activation_GetResult_service_members = {
  "dhtt_msgs::action",  // service namespace
  "Activation_GetResult",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<dhtt_msgs::action::Activation_GetResult>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t Activation_GetResult_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Activation_GetResult_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace dhtt_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<dhtt_msgs::action::Activation_GetResult>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::dhtt_msgs::action::rosidl_typesupport_introspection_cpp::Activation_GetResult_service_type_support_handle;
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
        ::dhtt_msgs::action::Activation_GetResult_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::dhtt_msgs::action::Activation_GetResult_Response
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
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dhtt_msgs, action, Activation_GetResult)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<dhtt_msgs::action::Activation_GetResult>();
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
// #include "dhtt_msgs/action/detail/activation__struct.hpp"
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

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void Activation_FeedbackMessage_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dhtt_msgs::action::Activation_FeedbackMessage(_init);
}

void Activation_FeedbackMessage_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dhtt_msgs::action::Activation_FeedbackMessage *>(message_memory);
  typed_message->~Activation_FeedbackMessage();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Activation_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<unique_identifier_msgs::msg::UUID>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::action::Activation_FeedbackMessage, goal_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "feedback",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dhtt_msgs::action::Activation_Feedback>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::action::Activation_FeedbackMessage, feedback),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Activation_FeedbackMessage_message_members = {
  "dhtt_msgs::action",  // message namespace
  "Activation_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(dhtt_msgs::action::Activation_FeedbackMessage),
  Activation_FeedbackMessage_message_member_array,  // message members
  Activation_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  Activation_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Activation_FeedbackMessage_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Activation_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace dhtt_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dhtt_msgs::action::Activation_FeedbackMessage>()
{
  return &::dhtt_msgs::action::rosidl_typesupport_introspection_cpp::Activation_FeedbackMessage_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dhtt_msgs, action, Activation_FeedbackMessage)() {
  return &::dhtt_msgs::action::rosidl_typesupport_introspection_cpp::Activation_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
