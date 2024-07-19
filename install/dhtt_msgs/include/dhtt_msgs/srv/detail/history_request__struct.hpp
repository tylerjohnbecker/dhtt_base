// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dhtt_msgs:srv/HistoryRequest.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__SRV__DETAIL__HISTORY_REQUEST__STRUCT_HPP_
#define DHTT_MSGS__SRV__DETAIL__HISTORY_REQUEST__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__dhtt_msgs__srv__HistoryRequest_Request __attribute__((deprecated))
#else
# define DEPRECATED__dhtt_msgs__srv__HistoryRequest_Request __declspec(deprecated)
#endif

namespace dhtt_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct HistoryRequest_Request_
{
  using Type = HistoryRequest_Request_<ContainerAllocator>;

  explicit HistoryRequest_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit HistoryRequest_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    dhtt_msgs::srv::HistoryRequest_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const dhtt_msgs::srv::HistoryRequest_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dhtt_msgs::srv::HistoryRequest_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dhtt_msgs::srv::HistoryRequest_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::srv::HistoryRequest_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::srv::HistoryRequest_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::srv::HistoryRequest_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::srv::HistoryRequest_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dhtt_msgs::srv::HistoryRequest_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dhtt_msgs::srv::HistoryRequest_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dhtt_msgs__srv__HistoryRequest_Request
    std::shared_ptr<dhtt_msgs::srv::HistoryRequest_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dhtt_msgs__srv__HistoryRequest_Request
    std::shared_ptr<dhtt_msgs::srv::HistoryRequest_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HistoryRequest_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const HistoryRequest_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HistoryRequest_Request_

// alias to use template instance with default allocator
using HistoryRequest_Request =
  dhtt_msgs::srv::HistoryRequest_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dhtt_msgs


#ifndef _WIN32
# define DEPRECATED__dhtt_msgs__srv__HistoryRequest_Response __attribute__((deprecated))
#else
# define DEPRECATED__dhtt_msgs__srv__HistoryRequest_Response __declspec(deprecated)
#endif

namespace dhtt_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct HistoryRequest_Response_
{
  using Type = HistoryRequest_Response_<ContainerAllocator>;

  explicit HistoryRequest_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit HistoryRequest_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _node_history_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _node_history_type node_history;

  // setters for named parameter idiom
  Type & set__node_history(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->node_history = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dhtt_msgs::srv::HistoryRequest_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const dhtt_msgs::srv::HistoryRequest_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dhtt_msgs::srv::HistoryRequest_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dhtt_msgs::srv::HistoryRequest_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::srv::HistoryRequest_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::srv::HistoryRequest_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::srv::HistoryRequest_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::srv::HistoryRequest_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dhtt_msgs::srv::HistoryRequest_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dhtt_msgs::srv::HistoryRequest_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dhtt_msgs__srv__HistoryRequest_Response
    std::shared_ptr<dhtt_msgs::srv::HistoryRequest_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dhtt_msgs__srv__HistoryRequest_Response
    std::shared_ptr<dhtt_msgs::srv::HistoryRequest_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HistoryRequest_Response_ & other) const
  {
    if (this->node_history != other.node_history) {
      return false;
    }
    return true;
  }
  bool operator!=(const HistoryRequest_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HistoryRequest_Response_

// alias to use template instance with default allocator
using HistoryRequest_Response =
  dhtt_msgs::srv::HistoryRequest_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dhtt_msgs

namespace dhtt_msgs
{

namespace srv
{

struct HistoryRequest
{
  using Request = dhtt_msgs::srv::HistoryRequest_Request;
  using Response = dhtt_msgs::srv::HistoryRequest_Response;
};

}  // namespace srv

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__SRV__DETAIL__HISTORY_REQUEST__STRUCT_HPP_
