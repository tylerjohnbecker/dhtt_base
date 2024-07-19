// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from dhtt_msgs:msg/Resources.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "dhtt_msgs/msg/detail/resources__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace dhtt_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Resources_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dhtt_msgs::msg::Resources(_init);
}

void Resources_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dhtt_msgs::msg::Resources *>(message_memory);
  typed_message->~Resources();
}

size_t size_function__Resources__resource_state(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<dhtt_msgs::msg::Resource> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Resources__resource_state(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<dhtt_msgs::msg::Resource> *>(untyped_member);
  return &member[index];
}

void * get_function__Resources__resource_state(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<dhtt_msgs::msg::Resource> *>(untyped_member);
  return &member[index];
}

void resize_function__Resources__resource_state(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<dhtt_msgs::msg::Resource> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Resources_message_member_array[1] = {
  {
    "resource_state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dhtt_msgs::msg::Resource>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::msg::Resources, resource_state),  // bytes offset in struct
    nullptr,  // default value
    size_function__Resources__resource_state,  // size() function pointer
    get_const_function__Resources__resource_state,  // get_const(index) function pointer
    get_function__Resources__resource_state,  // get(index) function pointer
    resize_function__Resources__resource_state  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Resources_message_members = {
  "dhtt_msgs::msg",  // message namespace
  "Resources",  // message name
  1,  // number of fields
  sizeof(dhtt_msgs::msg::Resources),
  Resources_message_member_array,  // message members
  Resources_init_function,  // function to initialize message memory (memory has to be allocated)
  Resources_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Resources_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Resources_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace dhtt_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dhtt_msgs::msg::Resources>()
{
  return &::dhtt_msgs::msg::rosidl_typesupport_introspection_cpp::Resources_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dhtt_msgs, msg, Resources)() {
  return &::dhtt_msgs::msg::rosidl_typesupport_introspection_cpp::Resources_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
