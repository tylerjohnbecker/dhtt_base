// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dhtt_msgs:msg/NodeStatus.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__MSG__DETAIL__NODE_STATUS__STRUCT_HPP_
#define DHTT_MSGS__MSG__DETAIL__NODE_STATUS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__dhtt_msgs__msg__NodeStatus __attribute__((deprecated))
#else
# define DEPRECATED__dhtt_msgs__msg__NodeStatus __declspec(deprecated)
#endif

namespace dhtt_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct NodeStatus_
{
  using Type = NodeStatus_<ContainerAllocator>;

  explicit NodeStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
    }
  }

  explicit NodeStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
    }
  }

  // field types and members
  using _state_type =
    int8_t;
  _state_type state;

  // setters for named parameter idiom
  Type & set__state(
    const int8_t & _arg)
  {
    this->state = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int8_t WAITING =
    1;
  static constexpr int8_t ACTIVE =
    2;
  static constexpr int8_t WORKING =
    3;
  static constexpr int8_t DONE =
    4;
  static constexpr int8_t MUTATING =
    5;

  // pointer types
  using RawPtr =
    dhtt_msgs::msg::NodeStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const dhtt_msgs::msg::NodeStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dhtt_msgs::msg::NodeStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dhtt_msgs::msg::NodeStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::msg::NodeStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::msg::NodeStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::msg::NodeStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::msg::NodeStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dhtt_msgs::msg::NodeStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dhtt_msgs::msg::NodeStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dhtt_msgs__msg__NodeStatus
    std::shared_ptr<dhtt_msgs::msg::NodeStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dhtt_msgs__msg__NodeStatus
    std::shared_ptr<dhtt_msgs::msg::NodeStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NodeStatus_ & other) const
  {
    if (this->state != other.state) {
      return false;
    }
    return true;
  }
  bool operator!=(const NodeStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NodeStatus_

// alias to use template instance with default allocator
using NodeStatus =
  dhtt_msgs::msg::NodeStatus_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr int8_t NodeStatus_<ContainerAllocator>::WAITING;
template<typename ContainerAllocator>
constexpr int8_t NodeStatus_<ContainerAllocator>::ACTIVE;
template<typename ContainerAllocator>
constexpr int8_t NodeStatus_<ContainerAllocator>::WORKING;
template<typename ContainerAllocator>
constexpr int8_t NodeStatus_<ContainerAllocator>::DONE;
template<typename ContainerAllocator>
constexpr int8_t NodeStatus_<ContainerAllocator>::MUTATING;

}  // namespace msg

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__MSG__DETAIL__NODE_STATUS__STRUCT_HPP_
