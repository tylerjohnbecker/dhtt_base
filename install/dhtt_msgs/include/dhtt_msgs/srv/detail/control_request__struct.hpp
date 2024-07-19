// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dhtt_msgs:srv/ControlRequest.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__SRV__DETAIL__CONTROL_REQUEST__STRUCT_HPP_
#define DHTT_MSGS__SRV__DETAIL__CONTROL_REQUEST__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__dhtt_msgs__srv__ControlRequest_Request __attribute__((deprecated))
#else
# define DEPRECATED__dhtt_msgs__srv__ControlRequest_Request __declspec(deprecated)
#endif

namespace dhtt_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ControlRequest_Request_
{
  using Type = ControlRequest_Request_<ContainerAllocator>;

  explicit ControlRequest_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0;
      this->file_name = "";
      this->interrupt = false;
    }
  }

  explicit ControlRequest_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : file_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0;
      this->file_name = "";
      this->interrupt = false;
    }
  }

  // field types and members
  using _type_type =
    int8_t;
  _type_type type;
  using _file_name_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _file_name_type file_name;
  using _interrupt_type =
    bool;
  _interrupt_type interrupt;

  // setters for named parameter idiom
  Type & set__type(
    const int8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__file_name(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->file_name = _arg;
    return *this;
  }
  Type & set__interrupt(
    const bool & _arg)
  {
    this->interrupt = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int8_t STOP =
    0;
  static constexpr int8_t START =
    1;
  static constexpr int8_t SAVE_TO_FILE =
    2;
  static constexpr int8_t RESET =
    3;

  // pointer types
  using RawPtr =
    dhtt_msgs::srv::ControlRequest_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const dhtt_msgs::srv::ControlRequest_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dhtt_msgs::srv::ControlRequest_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dhtt_msgs::srv::ControlRequest_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::srv::ControlRequest_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::srv::ControlRequest_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::srv::ControlRequest_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::srv::ControlRequest_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dhtt_msgs::srv::ControlRequest_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dhtt_msgs::srv::ControlRequest_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dhtt_msgs__srv__ControlRequest_Request
    std::shared_ptr<dhtt_msgs::srv::ControlRequest_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dhtt_msgs__srv__ControlRequest_Request
    std::shared_ptr<dhtt_msgs::srv::ControlRequest_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ControlRequest_Request_ & other) const
  {
    if (this->type != other.type) {
      return false;
    }
    if (this->file_name != other.file_name) {
      return false;
    }
    if (this->interrupt != other.interrupt) {
      return false;
    }
    return true;
  }
  bool operator!=(const ControlRequest_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ControlRequest_Request_

// alias to use template instance with default allocator
using ControlRequest_Request =
  dhtt_msgs::srv::ControlRequest_Request_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr int8_t ControlRequest_Request_<ContainerAllocator>::STOP;
template<typename ContainerAllocator>
constexpr int8_t ControlRequest_Request_<ContainerAllocator>::START;
template<typename ContainerAllocator>
constexpr int8_t ControlRequest_Request_<ContainerAllocator>::SAVE_TO_FILE;
template<typename ContainerAllocator>
constexpr int8_t ControlRequest_Request_<ContainerAllocator>::RESET;

}  // namespace srv

}  // namespace dhtt_msgs


#ifndef _WIN32
# define DEPRECATED__dhtt_msgs__srv__ControlRequest_Response __attribute__((deprecated))
#else
# define DEPRECATED__dhtt_msgs__srv__ControlRequest_Response __declspec(deprecated)
#endif

namespace dhtt_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ControlRequest_Response_
{
  using Type = ControlRequest_Response_<ContainerAllocator>;

  explicit ControlRequest_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->error_msg = "";
    }
  }

  explicit ControlRequest_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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

  // constant declarations

  // pointer types
  using RawPtr =
    dhtt_msgs::srv::ControlRequest_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const dhtt_msgs::srv::ControlRequest_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dhtt_msgs::srv::ControlRequest_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dhtt_msgs::srv::ControlRequest_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::srv::ControlRequest_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::srv::ControlRequest_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::srv::ControlRequest_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::srv::ControlRequest_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dhtt_msgs::srv::ControlRequest_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dhtt_msgs::srv::ControlRequest_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dhtt_msgs__srv__ControlRequest_Response
    std::shared_ptr<dhtt_msgs::srv::ControlRequest_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dhtt_msgs__srv__ControlRequest_Response
    std::shared_ptr<dhtt_msgs::srv::ControlRequest_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ControlRequest_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->error_msg != other.error_msg) {
      return false;
    }
    return true;
  }
  bool operator!=(const ControlRequest_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ControlRequest_Response_

// alias to use template instance with default allocator
using ControlRequest_Response =
  dhtt_msgs::srv::ControlRequest_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dhtt_msgs

namespace dhtt_msgs
{

namespace srv
{

struct ControlRequest
{
  using Request = dhtt_msgs::srv::ControlRequest_Request;
  using Response = dhtt_msgs::srv::ControlRequest_Response;
};

}  // namespace srv

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__SRV__DETAIL__CONTROL_REQUEST__STRUCT_HPP_
