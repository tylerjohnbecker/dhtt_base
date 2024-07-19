// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dhtt_msgs:srv/ModifyRequest.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__SRV__DETAIL__MODIFY_REQUEST__STRUCT_HPP_
#define DHTT_MSGS__SRV__DETAIL__MODIFY_REQUEST__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'add_node'
#include "dhtt_msgs/msg/detail/node__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dhtt_msgs__srv__ModifyRequest_Request __attribute__((deprecated))
#else
# define DEPRECATED__dhtt_msgs__srv__ModifyRequest_Request __declspec(deprecated)
#endif

namespace dhtt_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ModifyRequest_Request_
{
  using Type = ModifyRequest_Request_<ContainerAllocator>;

  explicit ModifyRequest_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : add_node(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0;
      this->to_add = "";
      this->mutate_type = "";
    }
  }

  explicit ModifyRequest_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : add_node(_alloc, _init),
    to_add(_alloc),
    mutate_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0;
      this->to_add = "";
      this->mutate_type = "";
    }
  }

  // field types and members
  using _type_type =
    int8_t;
  _type_type type;
  using _to_modify_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _to_modify_type to_modify;
  using _add_node_type =
    dhtt_msgs::msg::Node_<ContainerAllocator>;
  _add_node_type add_node;
  using _to_add_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _to_add_type to_add;
  using _params_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _params_type params;
  using _mutate_type_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _mutate_type_type mutate_type;

  // setters for named parameter idiom
  Type & set__type(
    const int8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__to_modify(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->to_modify = _arg;
    return *this;
  }
  Type & set__add_node(
    const dhtt_msgs::msg::Node_<ContainerAllocator> & _arg)
  {
    this->add_node = _arg;
    return *this;
  }
  Type & set__to_add(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->to_add = _arg;
    return *this;
  }
  Type & set__params(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->params = _arg;
    return *this;
  }
  Type & set__mutate_type(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->mutate_type = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int8_t ADD =
    0;
  static constexpr int8_t ADD_FROM_FILE =
    1;
  static constexpr int8_t REMOVE =
    2;
  static constexpr int8_t PARAM_UPDATE =
    3;
  static constexpr int8_t MUTATE =
    4;

  // pointer types
  using RawPtr =
    dhtt_msgs::srv::ModifyRequest_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const dhtt_msgs::srv::ModifyRequest_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dhtt_msgs::srv::ModifyRequest_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dhtt_msgs::srv::ModifyRequest_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::srv::ModifyRequest_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::srv::ModifyRequest_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::srv::ModifyRequest_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::srv::ModifyRequest_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dhtt_msgs::srv::ModifyRequest_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dhtt_msgs::srv::ModifyRequest_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dhtt_msgs__srv__ModifyRequest_Request
    std::shared_ptr<dhtt_msgs::srv::ModifyRequest_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dhtt_msgs__srv__ModifyRequest_Request
    std::shared_ptr<dhtt_msgs::srv::ModifyRequest_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ModifyRequest_Request_ & other) const
  {
    if (this->type != other.type) {
      return false;
    }
    if (this->to_modify != other.to_modify) {
      return false;
    }
    if (this->add_node != other.add_node) {
      return false;
    }
    if (this->to_add != other.to_add) {
      return false;
    }
    if (this->params != other.params) {
      return false;
    }
    if (this->mutate_type != other.mutate_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const ModifyRequest_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ModifyRequest_Request_

// alias to use template instance with default allocator
using ModifyRequest_Request =
  dhtt_msgs::srv::ModifyRequest_Request_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr int8_t ModifyRequest_Request_<ContainerAllocator>::ADD;
template<typename ContainerAllocator>
constexpr int8_t ModifyRequest_Request_<ContainerAllocator>::ADD_FROM_FILE;
template<typename ContainerAllocator>
constexpr int8_t ModifyRequest_Request_<ContainerAllocator>::REMOVE;
template<typename ContainerAllocator>
constexpr int8_t ModifyRequest_Request_<ContainerAllocator>::PARAM_UPDATE;
template<typename ContainerAllocator>
constexpr int8_t ModifyRequest_Request_<ContainerAllocator>::MUTATE;

}  // namespace srv

}  // namespace dhtt_msgs


#ifndef _WIN32
# define DEPRECATED__dhtt_msgs__srv__ModifyRequest_Response __attribute__((deprecated))
#else
# define DEPRECATED__dhtt_msgs__srv__ModifyRequest_Response __declspec(deprecated)
#endif

namespace dhtt_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ModifyRequest_Response_
{
  using Type = ModifyRequest_Response_<ContainerAllocator>;

  explicit ModifyRequest_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->error_msg = "";
    }
  }

  explicit ModifyRequest_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
  using _added_nodes_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _added_nodes_type added_nodes;
  using _removed_nodes_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _removed_nodes_type removed_nodes;

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
  Type & set__added_nodes(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->added_nodes = _arg;
    return *this;
  }
  Type & set__removed_nodes(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->removed_nodes = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dhtt_msgs::srv::ModifyRequest_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const dhtt_msgs::srv::ModifyRequest_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dhtt_msgs::srv::ModifyRequest_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dhtt_msgs::srv::ModifyRequest_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::srv::ModifyRequest_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::srv::ModifyRequest_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::srv::ModifyRequest_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::srv::ModifyRequest_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dhtt_msgs::srv::ModifyRequest_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dhtt_msgs::srv::ModifyRequest_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dhtt_msgs__srv__ModifyRequest_Response
    std::shared_ptr<dhtt_msgs::srv::ModifyRequest_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dhtt_msgs__srv__ModifyRequest_Response
    std::shared_ptr<dhtt_msgs::srv::ModifyRequest_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ModifyRequest_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->error_msg != other.error_msg) {
      return false;
    }
    if (this->added_nodes != other.added_nodes) {
      return false;
    }
    if (this->removed_nodes != other.removed_nodes) {
      return false;
    }
    return true;
  }
  bool operator!=(const ModifyRequest_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ModifyRequest_Response_

// alias to use template instance with default allocator
using ModifyRequest_Response =
  dhtt_msgs::srv::ModifyRequest_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dhtt_msgs

namespace dhtt_msgs
{

namespace srv
{

struct ModifyRequest
{
  using Request = dhtt_msgs::srv::ModifyRequest_Request;
  using Response = dhtt_msgs::srv::ModifyRequest_Response;
};

}  // namespace srv

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__SRV__DETAIL__MODIFY_REQUEST__STRUCT_HPP_
