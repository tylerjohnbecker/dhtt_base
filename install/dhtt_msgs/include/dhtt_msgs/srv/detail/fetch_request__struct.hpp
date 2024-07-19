// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dhtt_msgs:srv/FetchRequest.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__SRV__DETAIL__FETCH_REQUEST__STRUCT_HPP_
#define DHTT_MSGS__SRV__DETAIL__FETCH_REQUEST__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__dhtt_msgs__srv__FetchRequest_Request __attribute__((deprecated))
#else
# define DEPRECATED__dhtt_msgs__srv__FetchRequest_Request __declspec(deprecated)
#endif

namespace dhtt_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FetchRequest_Request_
{
  using Type = FetchRequest_Request_<ContainerAllocator>;

  explicit FetchRequest_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->return_full_subtree = false;
      this->common_name = "";
      this->node_name = "";
      this->node_type = 0;
    }
  }

  explicit FetchRequest_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : common_name(_alloc),
    node_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->return_full_subtree = false;
      this->common_name = "";
      this->node_name = "";
      this->node_type = 0;
    }
  }

  // field types and members
  using _return_full_subtree_type =
    bool;
  _return_full_subtree_type return_full_subtree;
  using _common_name_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _common_name_type common_name;
  using _node_name_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _node_name_type node_name;
  using _node_type_type =
    int8_t;
  _node_type_type node_type;

  // setters for named parameter idiom
  Type & set__return_full_subtree(
    const bool & _arg)
  {
    this->return_full_subtree = _arg;
    return *this;
  }
  Type & set__common_name(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->common_name = _arg;
    return *this;
  }
  Type & set__node_name(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->node_name = _arg;
    return *this;
  }
  Type & set__node_type(
    const int8_t & _arg)
  {
    this->node_type = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int8_t ROOT =
    1;
  static constexpr int8_t AND =
    2;
  static constexpr int8_t THEN =
    3;
  static constexpr int8_t OR =
    4;
  static constexpr int8_t BEHAVIOR =
    5;

  // pointer types
  using RawPtr =
    dhtt_msgs::srv::FetchRequest_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const dhtt_msgs::srv::FetchRequest_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dhtt_msgs::srv::FetchRequest_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dhtt_msgs::srv::FetchRequest_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::srv::FetchRequest_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::srv::FetchRequest_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::srv::FetchRequest_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::srv::FetchRequest_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dhtt_msgs::srv::FetchRequest_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dhtt_msgs::srv::FetchRequest_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dhtt_msgs__srv__FetchRequest_Request
    std::shared_ptr<dhtt_msgs::srv::FetchRequest_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dhtt_msgs__srv__FetchRequest_Request
    std::shared_ptr<dhtt_msgs::srv::FetchRequest_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FetchRequest_Request_ & other) const
  {
    if (this->return_full_subtree != other.return_full_subtree) {
      return false;
    }
    if (this->common_name != other.common_name) {
      return false;
    }
    if (this->node_name != other.node_name) {
      return false;
    }
    if (this->node_type != other.node_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const FetchRequest_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FetchRequest_Request_

// alias to use template instance with default allocator
using FetchRequest_Request =
  dhtt_msgs::srv::FetchRequest_Request_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr int8_t FetchRequest_Request_<ContainerAllocator>::ROOT;
template<typename ContainerAllocator>
constexpr int8_t FetchRequest_Request_<ContainerAllocator>::AND;
template<typename ContainerAllocator>
constexpr int8_t FetchRequest_Request_<ContainerAllocator>::THEN;
template<typename ContainerAllocator>
constexpr int8_t FetchRequest_Request_<ContainerAllocator>::OR;
template<typename ContainerAllocator>
constexpr int8_t FetchRequest_Request_<ContainerAllocator>::BEHAVIOR;

}  // namespace srv

}  // namespace dhtt_msgs


// Include directives for member types
// Member 'found_subtrees'
#include "dhtt_msgs/msg/detail/subtree__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dhtt_msgs__srv__FetchRequest_Response __attribute__((deprecated))
#else
# define DEPRECATED__dhtt_msgs__srv__FetchRequest_Response __declspec(deprecated)
#endif

namespace dhtt_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FetchRequest_Response_
{
  using Type = FetchRequest_Response_<ContainerAllocator>;

  explicit FetchRequest_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->error_msg = "";
    }
  }

  explicit FetchRequest_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : error_msg(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->error_msg = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _error_msg_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _error_msg_type error_msg;
  using _found_subtrees_type =
    std::vector<dhtt_msgs::msg::Subtree_<ContainerAllocator>, typename ContainerAllocator::template rebind<dhtt_msgs::msg::Subtree_<ContainerAllocator>>::other>;
  _found_subtrees_type found_subtrees;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__error_msg(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->error_msg = _arg;
    return *this;
  }
  Type & set__found_subtrees(
    const std::vector<dhtt_msgs::msg::Subtree_<ContainerAllocator>, typename ContainerAllocator::template rebind<dhtt_msgs::msg::Subtree_<ContainerAllocator>>::other> & _arg)
  {
    this->found_subtrees = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dhtt_msgs::srv::FetchRequest_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const dhtt_msgs::srv::FetchRequest_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dhtt_msgs::srv::FetchRequest_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dhtt_msgs::srv::FetchRequest_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::srv::FetchRequest_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::srv::FetchRequest_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::srv::FetchRequest_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::srv::FetchRequest_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dhtt_msgs::srv::FetchRequest_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dhtt_msgs::srv::FetchRequest_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dhtt_msgs__srv__FetchRequest_Response
    std::shared_ptr<dhtt_msgs::srv::FetchRequest_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dhtt_msgs__srv__FetchRequest_Response
    std::shared_ptr<dhtt_msgs::srv::FetchRequest_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FetchRequest_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->error_msg != other.error_msg) {
      return false;
    }
    if (this->found_subtrees != other.found_subtrees) {
      return false;
    }
    return true;
  }
  bool operator!=(const FetchRequest_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FetchRequest_Response_

// alias to use template instance with default allocator
using FetchRequest_Response =
  dhtt_msgs::srv::FetchRequest_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dhtt_msgs

namespace dhtt_msgs
{

namespace srv
{

struct FetchRequest
{
  using Request = dhtt_msgs::srv::FetchRequest_Request;
  using Response = dhtt_msgs::srv::FetchRequest_Response;
};

}  // namespace srv

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__SRV__DETAIL__FETCH_REQUEST__STRUCT_HPP_
