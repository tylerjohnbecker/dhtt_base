// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dhtt_msgs:srv/FetchInfo.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__SRV__DETAIL__FETCH_INFO__STRUCT_HPP_
#define DHTT_MSGS__SRV__DETAIL__FETCH_INFO__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__dhtt_msgs__srv__FetchInfo_Request __attribute__((deprecated))
#else
# define DEPRECATED__dhtt_msgs__srv__FetchInfo_Request __declspec(deprecated)
#endif

namespace dhtt_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FetchInfo_Request_
{
  using Type = FetchInfo_Request_<ContainerAllocator>;

  explicit FetchInfo_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit FetchInfo_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _keys_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _keys_type keys;

  // setters for named parameter idiom
  Type & set__keys(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->keys = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dhtt_msgs::srv::FetchInfo_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const dhtt_msgs::srv::FetchInfo_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dhtt_msgs::srv::FetchInfo_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dhtt_msgs::srv::FetchInfo_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::srv::FetchInfo_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::srv::FetchInfo_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::srv::FetchInfo_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::srv::FetchInfo_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dhtt_msgs::srv::FetchInfo_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dhtt_msgs::srv::FetchInfo_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dhtt_msgs__srv__FetchInfo_Request
    std::shared_ptr<dhtt_msgs::srv::FetchInfo_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dhtt_msgs__srv__FetchInfo_Request
    std::shared_ptr<dhtt_msgs::srv::FetchInfo_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FetchInfo_Request_ & other) const
  {
    if (this->keys != other.keys) {
      return false;
    }
    return true;
  }
  bool operator!=(const FetchInfo_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FetchInfo_Request_

// alias to use template instance with default allocator
using FetchInfo_Request =
  dhtt_msgs::srv::FetchInfo_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dhtt_msgs


// Include directives for member types
// Member 'information_pairs'
#include "dhtt_msgs/msg/detail/pair__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dhtt_msgs__srv__FetchInfo_Response __attribute__((deprecated))
#else
# define DEPRECATED__dhtt_msgs__srv__FetchInfo_Response __declspec(deprecated)
#endif

namespace dhtt_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FetchInfo_Response_
{
  using Type = FetchInfo_Response_<ContainerAllocator>;

  explicit FetchInfo_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit FetchInfo_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _information_pairs_type =
    std::vector<dhtt_msgs::msg::Pair_<ContainerAllocator>, typename ContainerAllocator::template rebind<dhtt_msgs::msg::Pair_<ContainerAllocator>>::other>;
  _information_pairs_type information_pairs;

  // setters for named parameter idiom
  Type & set__information_pairs(
    const std::vector<dhtt_msgs::msg::Pair_<ContainerAllocator>, typename ContainerAllocator::template rebind<dhtt_msgs::msg::Pair_<ContainerAllocator>>::other> & _arg)
  {
    this->information_pairs = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dhtt_msgs::srv::FetchInfo_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const dhtt_msgs::srv::FetchInfo_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dhtt_msgs::srv::FetchInfo_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dhtt_msgs::srv::FetchInfo_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::srv::FetchInfo_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::srv::FetchInfo_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::srv::FetchInfo_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::srv::FetchInfo_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dhtt_msgs::srv::FetchInfo_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dhtt_msgs::srv::FetchInfo_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dhtt_msgs__srv__FetchInfo_Response
    std::shared_ptr<dhtt_msgs::srv::FetchInfo_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dhtt_msgs__srv__FetchInfo_Response
    std::shared_ptr<dhtt_msgs::srv::FetchInfo_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FetchInfo_Response_ & other) const
  {
    if (this->information_pairs != other.information_pairs) {
      return false;
    }
    return true;
  }
  bool operator!=(const FetchInfo_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FetchInfo_Response_

// alias to use template instance with default allocator
using FetchInfo_Response =
  dhtt_msgs::srv::FetchInfo_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dhtt_msgs

namespace dhtt_msgs
{

namespace srv
{

struct FetchInfo
{
  using Request = dhtt_msgs::srv::FetchInfo_Request;
  using Response = dhtt_msgs::srv::FetchInfo_Response;
};

}  // namespace srv

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__SRV__DETAIL__FETCH_INFO__STRUCT_HPP_
