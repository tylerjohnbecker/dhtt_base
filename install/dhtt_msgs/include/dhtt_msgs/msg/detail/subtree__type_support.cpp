// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from dhtt_msgs:msg/Subtree.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "dhtt_msgs/msg/detail/subtree__struct.hpp"
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

void Subtree_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dhtt_msgs::msg::Subtree(_init);
}

void Subtree_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dhtt_msgs::msg::Subtree *>(message_memory);
  typed_message->~Subtree();
}

size_t size_function__Subtree__tree_nodes(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<dhtt_msgs::msg::Node> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Subtree__tree_nodes(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<dhtt_msgs::msg::Node> *>(untyped_member);
  return &member[index];
}

void * get_function__Subtree__tree_nodes(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<dhtt_msgs::msg::Node> *>(untyped_member);
  return &member[index];
}

void resize_function__Subtree__tree_nodes(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<dhtt_msgs::msg::Node> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Subtree_message_member_array[5] = {
  {
    "tree_nodes",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dhtt_msgs::msg::Node>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::msg::Subtree, tree_nodes),  // bytes offset in struct
    nullptr,  // default value
    size_function__Subtree__tree_nodes,  // size() function pointer
    get_const_function__Subtree__tree_nodes,  // get_const(index) function pointer
    get_function__Subtree__tree_nodes,  // get(index) function pointer
    resize_function__Subtree__tree_nodes  // resize(index) function pointer
  },
  {
    "tree_status",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::msg::Subtree, tree_status),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "max_tree_depth",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::msg::Subtree, max_tree_depth),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "max_tree_width",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::msg::Subtree, max_tree_width),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "task_completion_percent",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dhtt_msgs::msg::Subtree, task_completion_percent),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Subtree_message_members = {
  "dhtt_msgs::msg",  // message namespace
  "Subtree",  // message name
  5,  // number of fields
  sizeof(dhtt_msgs::msg::Subtree),
  Subtree_message_member_array,  // message members
  Subtree_init_function,  // function to initialize message memory (memory has to be allocated)
  Subtree_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Subtree_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Subtree_message_members,
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
get_message_type_support_handle<dhtt_msgs::msg::Subtree>()
{
  return &::dhtt_msgs::msg::rosidl_typesupport_introspection_cpp::Subtree_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dhtt_msgs, msg, Subtree)() {
  return &::dhtt_msgs::msg::rosidl_typesupport_introspection_cpp::Subtree_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
