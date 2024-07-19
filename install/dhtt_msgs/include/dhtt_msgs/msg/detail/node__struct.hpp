// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dhtt_msgs:msg/Node.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__MSG__DETAIL__NODE__STRUCT_HPP_
#define DHTT_MSGS__MSG__DETAIL__NODE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'head'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'owned_resources'
#include "dhtt_msgs/msg/detail/resource__struct.hpp"
// Member 'node_status'
#include "dhtt_msgs/msg/detail/node_status__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dhtt_msgs__msg__Node __attribute__((deprecated))
#else
# define DEPRECATED__dhtt_msgs__msg__Node __declspec(deprecated)
#endif

namespace dhtt_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Node_
{
  using Type = Node_<ContainerAllocator>;

  explicit Node_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : head(_init),
    node_status(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->node_name = "";
      this->parent = 0l;
      this->parent_name = "";
      this->type = 0;
      this->plugin_name = "";
    }
  }

  explicit Node_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : head(_alloc, _init),
    node_name(_alloc),
    parent_name(_alloc),
    plugin_name(_alloc),
    node_status(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->node_name = "";
      this->parent = 0l;
      this->parent_name = "";
      this->type = 0;
      this->plugin_name = "";
    }
  }

  // field types and members
  using _head_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _head_type head;
  using _node_name_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _node_name_type node_name;
  using _parent_type =
    int32_t;
  _parent_type parent;
  using _parent_name_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _parent_name_type parent_name;
  using _children_type =
    std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other>;
  _children_type children;
  using _child_name_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _child_name_type child_name;
  using _params_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _params_type params;
  using _type_type =
    int8_t;
  _type_type type;
  using _plugin_name_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _plugin_name_type plugin_name;
  using _owned_resources_type =
    std::vector<dhtt_msgs::msg::Resource_<ContainerAllocator>, typename ContainerAllocator::template rebind<dhtt_msgs::msg::Resource_<ContainerAllocator>>::other>;
  _owned_resources_type owned_resources;
  using _node_status_type =
    dhtt_msgs::msg::NodeStatus_<ContainerAllocator>;
  _node_status_type node_status;

  // setters for named parameter idiom
  Type & set__head(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->head = _arg;
    return *this;
  }
  Type & set__node_name(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->node_name = _arg;
    return *this;
  }
  Type & set__parent(
    const int32_t & _arg)
  {
    this->parent = _arg;
    return *this;
  }
  Type & set__parent_name(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->parent_name = _arg;
    return *this;
  }
  Type & set__children(
    const std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other> & _arg)
  {
    this->children = _arg;
    return *this;
  }
  Type & set__child_name(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->child_name = _arg;
    return *this;
  }
  Type & set__params(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->params = _arg;
    return *this;
  }
  Type & set__type(
    const int8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__plugin_name(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->plugin_name = _arg;
    return *this;
  }
  Type & set__owned_resources(
    const std::vector<dhtt_msgs::msg::Resource_<ContainerAllocator>, typename ContainerAllocator::template rebind<dhtt_msgs::msg::Resource_<ContainerAllocator>>::other> & _arg)
  {
    this->owned_resources = _arg;
    return *this;
  }
  Type & set__node_status(
    const dhtt_msgs::msg::NodeStatus_<ContainerAllocator> & _arg)
  {
    this->node_status = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int8_t ROOT =
    0;
  static constexpr int8_t AND =
    1;
  static constexpr int8_t THEN =
    2;
  static constexpr int8_t OR =
    3;
  static constexpr int8_t BEHAVIOR =
    4;

  // pointer types
  using RawPtr =
    dhtt_msgs::msg::Node_<ContainerAllocator> *;
  using ConstRawPtr =
    const dhtt_msgs::msg::Node_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dhtt_msgs::msg::Node_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dhtt_msgs::msg::Node_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::msg::Node_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::msg::Node_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::msg::Node_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::msg::Node_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dhtt_msgs::msg::Node_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dhtt_msgs::msg::Node_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dhtt_msgs__msg__Node
    std::shared_ptr<dhtt_msgs::msg::Node_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dhtt_msgs__msg__Node
    std::shared_ptr<dhtt_msgs::msg::Node_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Node_ & other) const
  {
    if (this->head != other.head) {
      return false;
    }
    if (this->node_name != other.node_name) {
      return false;
    }
    if (this->parent != other.parent) {
      return false;
    }
    if (this->parent_name != other.parent_name) {
      return false;
    }
    if (this->children != other.children) {
      return false;
    }
    if (this->child_name != other.child_name) {
      return false;
    }
    if (this->params != other.params) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    if (this->plugin_name != other.plugin_name) {
      return false;
    }
    if (this->owned_resources != other.owned_resources) {
      return false;
    }
    if (this->node_status != other.node_status) {
      return false;
    }
    return true;
  }
  bool operator!=(const Node_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Node_

// alias to use template instance with default allocator
using Node =
  dhtt_msgs::msg::Node_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr int8_t Node_<ContainerAllocator>::ROOT;
template<typename ContainerAllocator>
constexpr int8_t Node_<ContainerAllocator>::AND;
template<typename ContainerAllocator>
constexpr int8_t Node_<ContainerAllocator>::THEN;
template<typename ContainerAllocator>
constexpr int8_t Node_<ContainerAllocator>::OR;
template<typename ContainerAllocator>
constexpr int8_t Node_<ContainerAllocator>::BEHAVIOR;

}  // namespace msg

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__MSG__DETAIL__NODE__STRUCT_HPP_
