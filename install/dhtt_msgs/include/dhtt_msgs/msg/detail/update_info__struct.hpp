// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dhtt_msgs:msg/UpdateInfo.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__MSG__DETAIL__UPDATE_INFO__STRUCT_HPP_
#define DHTT_MSGS__MSG__DETAIL__UPDATE_INFO__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'updates'
#include "dhtt_msgs/msg/detail/pair__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dhtt_msgs__msg__UpdateInfo __attribute__((deprecated))
#else
# define DEPRECATED__dhtt_msgs__msg__UpdateInfo __declspec(deprecated)
#endif

namespace dhtt_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UpdateInfo_
{
  using Type = UpdateInfo_<ContainerAllocator>;

  explicit UpdateInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit UpdateInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _updates_type =
    std::vector<dhtt_msgs::msg::Pair_<ContainerAllocator>, typename ContainerAllocator::template rebind<dhtt_msgs::msg::Pair_<ContainerAllocator>>::other>;
  _updates_type updates;

  // setters for named parameter idiom
  Type & set__updates(
    const std::vector<dhtt_msgs::msg::Pair_<ContainerAllocator>, typename ContainerAllocator::template rebind<dhtt_msgs::msg::Pair_<ContainerAllocator>>::other> & _arg)
  {
    this->updates = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dhtt_msgs::msg::UpdateInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const dhtt_msgs::msg::UpdateInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dhtt_msgs::msg::UpdateInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dhtt_msgs::msg::UpdateInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::msg::UpdateInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::msg::UpdateInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::msg::UpdateInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::msg::UpdateInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dhtt_msgs::msg::UpdateInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dhtt_msgs::msg::UpdateInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dhtt_msgs__msg__UpdateInfo
    std::shared_ptr<dhtt_msgs::msg::UpdateInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dhtt_msgs__msg__UpdateInfo
    std::shared_ptr<dhtt_msgs::msg::UpdateInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UpdateInfo_ & other) const
  {
    if (this->updates != other.updates) {
      return false;
    }
    return true;
  }
  bool operator!=(const UpdateInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UpdateInfo_

// alias to use template instance with default allocator
using UpdateInfo =
  dhtt_msgs::msg::UpdateInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__MSG__DETAIL__UPDATE_INFO__STRUCT_HPP_
