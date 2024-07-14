// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from knowledge_server:msg/Update.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "knowledge_server/msg/detail/update__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace knowledge_server
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Update_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) knowledge_server::msg::Update(_init);
}

void Update_fini_function(void * message_memory)
{
  auto typed_message = static_cast<knowledge_server::msg::Update *>(message_memory);
  typed_message->~Update();
}

size_t size_function__Update__updated_pairs(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<knowledge_server::msg::Pair> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Update__updated_pairs(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<knowledge_server::msg::Pair> *>(untyped_member);
  return &member[index];
}

void * get_function__Update__updated_pairs(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<knowledge_server::msg::Pair> *>(untyped_member);
  return &member[index];
}

void resize_function__Update__updated_pairs(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<knowledge_server::msg::Pair> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Update_message_member_array[1] = {
  {
    "updated_pairs",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<knowledge_server::msg::Pair>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(knowledge_server::msg::Update, updated_pairs),  // bytes offset in struct
    nullptr,  // default value
    size_function__Update__updated_pairs,  // size() function pointer
    get_const_function__Update__updated_pairs,  // get_const(index) function pointer
    get_function__Update__updated_pairs,  // get(index) function pointer
    resize_function__Update__updated_pairs  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Update_message_members = {
  "knowledge_server::msg",  // message namespace
  "Update",  // message name
  1,  // number of fields
  sizeof(knowledge_server::msg::Update),
  Update_message_member_array,  // message members
  Update_init_function,  // function to initialize message memory (memory has to be allocated)
  Update_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Update_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Update_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace knowledge_server


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<knowledge_server::msg::Update>()
{
  return &::knowledge_server::msg::rosidl_typesupport_introspection_cpp::Update_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, knowledge_server, msg, Update)() {
  return &::knowledge_server::msg::rosidl_typesupport_introspection_cpp::Update_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
