// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dhtt_msgs:srv/GoitrRequest.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__SRV__DETAIL__GOITR_REQUEST__STRUCT_HPP_
#define DHTT_MSGS__SRV__DETAIL__GOITR_REQUEST__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__dhtt_msgs__srv__GoitrRequest_Request __attribute__((deprecated))
#else
# define DEPRECATED__dhtt_msgs__srv__GoitrRequest_Request __declspec(deprecated)
#endif

namespace dhtt_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GoitrRequest_Request_
{
  using Type = GoitrRequest_Request_<ContainerAllocator>;

  explicit GoitrRequest_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0;
    }
  }

  explicit GoitrRequest_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0;
    }
  }

  // field types and members
  using _type_type =
    int8_t;
  _type_type type;
  using _params_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _params_type params;

  // setters for named parameter idiom
  Type & set__type(
    const int8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__params(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->params = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int8_t GENERAL =
    0;

  // pointer types
  using RawPtr =
    dhtt_msgs::srv::GoitrRequest_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const dhtt_msgs::srv::GoitrRequest_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dhtt_msgs::srv::GoitrRequest_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dhtt_msgs::srv::GoitrRequest_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::srv::GoitrRequest_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::srv::GoitrRequest_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::srv::GoitrRequest_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::srv::GoitrRequest_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dhtt_msgs::srv::GoitrRequest_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dhtt_msgs::srv::GoitrRequest_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dhtt_msgs__srv__GoitrRequest_Request
    std::shared_ptr<dhtt_msgs::srv::GoitrRequest_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dhtt_msgs__srv__GoitrRequest_Request
    std::shared_ptr<dhtt_msgs::srv::GoitrRequest_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GoitrRequest_Request_ & other) const
  {
    if (this->type != other.type) {
      return false;
    }
    if (this->params != other.params) {
      return false;
    }
    return true;
  }
  bool operator!=(const GoitrRequest_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GoitrRequest_Request_

// alias to use template instance with default allocator
using GoitrRequest_Request =
  dhtt_msgs::srv::GoitrRequest_Request_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr int8_t GoitrRequest_Request_<ContainerAllocator>::GENERAL;

}  // namespace srv

}  // namespace dhtt_msgs


#ifndef _WIN32
# define DEPRECATED__dhtt_msgs__srv__GoitrRequest_Response __attribute__((deprecated))
#else
# define DEPRECATED__dhtt_msgs__srv__GoitrRequest_Response __declspec(deprecated)
#endif

namespace dhtt_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GoitrRequest_Response_
{
  using Type = GoitrRequest_Response_<ContainerAllocator>;

  explicit GoitrRequest_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->error_msgs = "";
    }
  }

  explicit GoitrRequest_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : error_msgs(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->error_msgs = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _error_msgs_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _error_msgs_type error_msgs;
  using _responses_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _responses_type responses;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__error_msgs(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->error_msgs = _arg;
    return *this;
  }
  Type & set__responses(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->responses = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dhtt_msgs::srv::GoitrRequest_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const dhtt_msgs::srv::GoitrRequest_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dhtt_msgs::srv::GoitrRequest_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dhtt_msgs::srv::GoitrRequest_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::srv::GoitrRequest_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::srv::GoitrRequest_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::srv::GoitrRequest_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::srv::GoitrRequest_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dhtt_msgs::srv::GoitrRequest_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dhtt_msgs::srv::GoitrRequest_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dhtt_msgs__srv__GoitrRequest_Response
    std::shared_ptr<dhtt_msgs::srv::GoitrRequest_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dhtt_msgs__srv__GoitrRequest_Response
    std::shared_ptr<dhtt_msgs::srv::GoitrRequest_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GoitrRequest_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->error_msgs != other.error_msgs) {
      return false;
    }
    if (this->responses != other.responses) {
      return false;
    }
    return true;
  }
  bool operator!=(const GoitrRequest_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GoitrRequest_Response_

// alias to use template instance with default allocator
using GoitrRequest_Response =
  dhtt_msgs::srv::GoitrRequest_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dhtt_msgs

namespace dhtt_msgs
{

namespace srv
{

struct GoitrRequest
{
  using Request = dhtt_msgs::srv::GoitrRequest_Request;
  using Response = dhtt_msgs::srv::GoitrRequest_Response;
};

}  // namespace srv

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__SRV__DETAIL__GOITR_REQUEST__STRUCT_HPP_
