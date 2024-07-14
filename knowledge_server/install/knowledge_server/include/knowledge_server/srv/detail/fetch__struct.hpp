// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from knowledge_server:srv/Fetch.idl
// generated code does not contain a copyright notice

#ifndef KNOWLEDGE_SERVER__SRV__DETAIL__FETCH__STRUCT_HPP_
#define KNOWLEDGE_SERVER__SRV__DETAIL__FETCH__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__knowledge_server__srv__Fetch_Request __attribute__((deprecated))
#else
# define DEPRECATED__knowledge_server__srv__Fetch_Request __declspec(deprecated)
#endif

namespace knowledge_server
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Fetch_Request_
{
  using Type = Fetch_Request_<ContainerAllocator>;

  explicit Fetch_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Fetch_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    knowledge_server::srv::Fetch_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const knowledge_server::srv::Fetch_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<knowledge_server::srv::Fetch_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<knowledge_server::srv::Fetch_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      knowledge_server::srv::Fetch_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<knowledge_server::srv::Fetch_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      knowledge_server::srv::Fetch_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<knowledge_server::srv::Fetch_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<knowledge_server::srv::Fetch_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<knowledge_server::srv::Fetch_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__knowledge_server__srv__Fetch_Request
    std::shared_ptr<knowledge_server::srv::Fetch_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__knowledge_server__srv__Fetch_Request
    std::shared_ptr<knowledge_server::srv::Fetch_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Fetch_Request_ & other) const
  {
    if (this->keys != other.keys) {
      return false;
    }
    return true;
  }
  bool operator!=(const Fetch_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Fetch_Request_

// alias to use template instance with default allocator
using Fetch_Request =
  knowledge_server::srv::Fetch_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace knowledge_server


// Include directives for member types
// Member 'pairs'
#include "knowledge_server/msg/detail/pair__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__knowledge_server__srv__Fetch_Response __attribute__((deprecated))
#else
# define DEPRECATED__knowledge_server__srv__Fetch_Response __declspec(deprecated)
#endif

namespace knowledge_server
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Fetch_Response_
{
  using Type = Fetch_Response_<ContainerAllocator>;

  explicit Fetch_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Fetch_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _pairs_type =
    std::vector<knowledge_server::msg::Pair_<ContainerAllocator>, typename ContainerAllocator::template rebind<knowledge_server::msg::Pair_<ContainerAllocator>>::other>;
  _pairs_type pairs;

  // setters for named parameter idiom
  Type & set__pairs(
    const std::vector<knowledge_server::msg::Pair_<ContainerAllocator>, typename ContainerAllocator::template rebind<knowledge_server::msg::Pair_<ContainerAllocator>>::other> & _arg)
  {
    this->pairs = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    knowledge_server::srv::Fetch_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const knowledge_server::srv::Fetch_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<knowledge_server::srv::Fetch_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<knowledge_server::srv::Fetch_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      knowledge_server::srv::Fetch_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<knowledge_server::srv::Fetch_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      knowledge_server::srv::Fetch_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<knowledge_server::srv::Fetch_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<knowledge_server::srv::Fetch_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<knowledge_server::srv::Fetch_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__knowledge_server__srv__Fetch_Response
    std::shared_ptr<knowledge_server::srv::Fetch_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__knowledge_server__srv__Fetch_Response
    std::shared_ptr<knowledge_server::srv::Fetch_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Fetch_Response_ & other) const
  {
    if (this->pairs != other.pairs) {
      return false;
    }
    return true;
  }
  bool operator!=(const Fetch_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Fetch_Response_

// alias to use template instance with default allocator
using Fetch_Response =
  knowledge_server::srv::Fetch_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace knowledge_server

namespace knowledge_server
{

namespace srv
{

struct Fetch
{
  using Request = knowledge_server::srv::Fetch_Request;
  using Response = knowledge_server::srv::Fetch_Response;
};

}  // namespace srv

}  // namespace knowledge_server

#endif  // KNOWLEDGE_SERVER__SRV__DETAIL__FETCH__STRUCT_HPP_
