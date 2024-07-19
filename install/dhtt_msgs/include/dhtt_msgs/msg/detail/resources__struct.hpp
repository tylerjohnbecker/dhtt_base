// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dhtt_msgs:msg/Resources.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__MSG__DETAIL__RESOURCES__STRUCT_HPP_
#define DHTT_MSGS__MSG__DETAIL__RESOURCES__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'resource_state'
#include "dhtt_msgs/msg/detail/resource__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dhtt_msgs__msg__Resources __attribute__((deprecated))
#else
# define DEPRECATED__dhtt_msgs__msg__Resources __declspec(deprecated)
#endif

namespace dhtt_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Resources_
{
  using Type = Resources_<ContainerAllocator>;

  explicit Resources_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Resources_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _resource_state_type =
    std::vector<dhtt_msgs::msg::Resource_<ContainerAllocator>, typename ContainerAllocator::template rebind<dhtt_msgs::msg::Resource_<ContainerAllocator>>::other>;
  _resource_state_type resource_state;

  // setters for named parameter idiom
  Type & set__resource_state(
    const std::vector<dhtt_msgs::msg::Resource_<ContainerAllocator>, typename ContainerAllocator::template rebind<dhtt_msgs::msg::Resource_<ContainerAllocator>>::other> & _arg)
  {
    this->resource_state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dhtt_msgs::msg::Resources_<ContainerAllocator> *;
  using ConstRawPtr =
    const dhtt_msgs::msg::Resources_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dhtt_msgs::msg::Resources_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dhtt_msgs::msg::Resources_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::msg::Resources_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::msg::Resources_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::msg::Resources_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::msg::Resources_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dhtt_msgs::msg::Resources_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dhtt_msgs::msg::Resources_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dhtt_msgs__msg__Resources
    std::shared_ptr<dhtt_msgs::msg::Resources_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dhtt_msgs__msg__Resources
    std::shared_ptr<dhtt_msgs::msg::Resources_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Resources_ & other) const
  {
    if (this->resource_state != other.resource_state) {
      return false;
    }
    return true;
  }
  bool operator!=(const Resources_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Resources_

// alias to use template instance with default allocator
using Resources =
  dhtt_msgs::msg::Resources_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__MSG__DETAIL__RESOURCES__STRUCT_HPP_
