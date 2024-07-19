// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dhtt_msgs:srv/InternalServiceRegistration.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__SRV__DETAIL__INTERNAL_SERVICE_REGISTRATION__STRUCT_HPP_
#define DHTT_MSGS__SRV__DETAIL__INTERNAL_SERVICE_REGISTRATION__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__dhtt_msgs__srv__InternalServiceRegistration_Request __attribute__((deprecated))
#else
# define DEPRECATED__dhtt_msgs__srv__InternalServiceRegistration_Request __declspec(deprecated)
#endif

namespace dhtt_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct InternalServiceRegistration_Request_
{
  using Type = InternalServiceRegistration_Request_<ContainerAllocator>;

  explicit InternalServiceRegistration_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->node_name = "";
    }
  }

  explicit InternalServiceRegistration_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : node_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->node_name = "";
    }
  }

  // field types and members
  using _node_name_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _node_name_type node_name;

  // setters for named parameter idiom
  Type & set__node_name(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->node_name = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dhtt_msgs::srv::InternalServiceRegistration_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const dhtt_msgs::srv::InternalServiceRegistration_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::srv::InternalServiceRegistration_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::srv::InternalServiceRegistration_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::srv::InternalServiceRegistration_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::srv::InternalServiceRegistration_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dhtt_msgs::srv::InternalServiceRegistration_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dhtt_msgs::srv::InternalServiceRegistration_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dhtt_msgs__srv__InternalServiceRegistration_Request
    std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dhtt_msgs__srv__InternalServiceRegistration_Request
    std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InternalServiceRegistration_Request_ & other) const
  {
    if (this->node_name != other.node_name) {
      return false;
    }
    return true;
  }
  bool operator!=(const InternalServiceRegistration_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InternalServiceRegistration_Request_

// alias to use template instance with default allocator
using InternalServiceRegistration_Request =
  dhtt_msgs::srv::InternalServiceRegistration_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dhtt_msgs


#ifndef _WIN32
# define DEPRECATED__dhtt_msgs__srv__InternalServiceRegistration_Response __attribute__((deprecated))
#else
# define DEPRECATED__dhtt_msgs__srv__InternalServiceRegistration_Response __declspec(deprecated)
#endif

namespace dhtt_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct InternalServiceRegistration_Response_
{
  using Type = InternalServiceRegistration_Response_<ContainerAllocator>;

  explicit InternalServiceRegistration_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit InternalServiceRegistration_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dhtt_msgs::srv::InternalServiceRegistration_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const dhtt_msgs::srv::InternalServiceRegistration_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::srv::InternalServiceRegistration_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::srv::InternalServiceRegistration_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::srv::InternalServiceRegistration_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::srv::InternalServiceRegistration_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dhtt_msgs::srv::InternalServiceRegistration_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dhtt_msgs::srv::InternalServiceRegistration_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dhtt_msgs__srv__InternalServiceRegistration_Response
    std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dhtt_msgs__srv__InternalServiceRegistration_Response
    std::shared_ptr<dhtt_msgs::srv::InternalServiceRegistration_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InternalServiceRegistration_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const InternalServiceRegistration_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InternalServiceRegistration_Response_

// alias to use template instance with default allocator
using InternalServiceRegistration_Response =
  dhtt_msgs::srv::InternalServiceRegistration_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dhtt_msgs

namespace dhtt_msgs
{

namespace srv
{

struct InternalServiceRegistration
{
  using Request = dhtt_msgs::srv::InternalServiceRegistration_Request;
  using Response = dhtt_msgs::srv::InternalServiceRegistration_Response;
};

}  // namespace srv

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__SRV__DETAIL__INTERNAL_SERVICE_REGISTRATION__STRUCT_HPP_
