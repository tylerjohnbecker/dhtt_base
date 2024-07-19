// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dhtt_msgs:msg/Subtree.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__MSG__DETAIL__SUBTREE__STRUCT_HPP_
#define DHTT_MSGS__MSG__DETAIL__SUBTREE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'tree_nodes'
#include "dhtt_msgs/msg/detail/node__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dhtt_msgs__msg__Subtree __attribute__((deprecated))
#else
# define DEPRECATED__dhtt_msgs__msg__Subtree __declspec(deprecated)
#endif

namespace dhtt_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Subtree_
{
  using Type = Subtree_<ContainerAllocator>;

  explicit Subtree_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->tree_status = 0;
      this->max_tree_depth = 0l;
      this->max_tree_width = 0l;
      this->task_completion_percent = 0.0f;
    }
  }

  explicit Subtree_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->tree_status = 0;
      this->max_tree_depth = 0l;
      this->max_tree_width = 0l;
      this->task_completion_percent = 0.0f;
    }
  }

  // field types and members
  using _tree_nodes_type =
    std::vector<dhtt_msgs::msg::Node_<ContainerAllocator>, typename ContainerAllocator::template rebind<dhtt_msgs::msg::Node_<ContainerAllocator>>::other>;
  _tree_nodes_type tree_nodes;
  using _tree_status_type =
    int8_t;
  _tree_status_type tree_status;
  using _max_tree_depth_type =
    int32_t;
  _max_tree_depth_type max_tree_depth;
  using _max_tree_width_type =
    int32_t;
  _max_tree_width_type max_tree_width;
  using _task_completion_percent_type =
    float;
  _task_completion_percent_type task_completion_percent;

  // setters for named parameter idiom
  Type & set__tree_nodes(
    const std::vector<dhtt_msgs::msg::Node_<ContainerAllocator>, typename ContainerAllocator::template rebind<dhtt_msgs::msg::Node_<ContainerAllocator>>::other> & _arg)
  {
    this->tree_nodes = _arg;
    return *this;
  }
  Type & set__tree_status(
    const int8_t & _arg)
  {
    this->tree_status = _arg;
    return *this;
  }
  Type & set__max_tree_depth(
    const int32_t & _arg)
  {
    this->max_tree_depth = _arg;
    return *this;
  }
  Type & set__max_tree_width(
    const int32_t & _arg)
  {
    this->max_tree_width = _arg;
    return *this;
  }
  Type & set__task_completion_percent(
    const float & _arg)
  {
    this->task_completion_percent = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dhtt_msgs::msg::Subtree_<ContainerAllocator> *;
  using ConstRawPtr =
    const dhtt_msgs::msg::Subtree_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dhtt_msgs::msg::Subtree_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dhtt_msgs::msg::Subtree_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::msg::Subtree_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::msg::Subtree_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::msg::Subtree_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::msg::Subtree_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dhtt_msgs::msg::Subtree_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dhtt_msgs::msg::Subtree_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dhtt_msgs__msg__Subtree
    std::shared_ptr<dhtt_msgs::msg::Subtree_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dhtt_msgs__msg__Subtree
    std::shared_ptr<dhtt_msgs::msg::Subtree_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Subtree_ & other) const
  {
    if (this->tree_nodes != other.tree_nodes) {
      return false;
    }
    if (this->tree_status != other.tree_status) {
      return false;
    }
    if (this->max_tree_depth != other.max_tree_depth) {
      return false;
    }
    if (this->max_tree_width != other.max_tree_width) {
      return false;
    }
    if (this->task_completion_percent != other.task_completion_percent) {
      return false;
    }
    return true;
  }
  bool operator!=(const Subtree_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Subtree_

// alias to use template instance with default allocator
using Subtree =
  dhtt_msgs::msg::Subtree_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__MSG__DETAIL__SUBTREE__STRUCT_HPP_
