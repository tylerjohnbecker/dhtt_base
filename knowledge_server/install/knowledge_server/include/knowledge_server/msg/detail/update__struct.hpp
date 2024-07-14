// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from knowledge_server:msg/Update.idl
// generated code does not contain a copyright notice

#ifndef KNOWLEDGE_SERVER__MSG__DETAIL__UPDATE__STRUCT_HPP_
#define KNOWLEDGE_SERVER__MSG__DETAIL__UPDATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'updated_pairs'
#include "knowledge_server/msg/detail/pair__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__knowledge_server__msg__Update __attribute__((deprecated))
#else
# define DEPRECATED__knowledge_server__msg__Update __declspec(deprecated)
#endif

namespace knowledge_server
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Update_
{
  using Type = Update_<ContainerAllocator>;

  explicit Update_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Update_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _updated_pairs_type =
    std::vector<knowledge_server::msg::Pair_<ContainerAllocator>, typename ContainerAllocator::template rebind<knowledge_server::msg::Pair_<ContainerAllocator>>::other>;
  _updated_pairs_type updated_pairs;

  // setters for named parameter idiom
  Type & set__updated_pairs(
    const std::vector<knowledge_server::msg::Pair_<ContainerAllocator>, typename ContainerAllocator::template rebind<knowledge_server::msg::Pair_<ContainerAllocator>>::other> & _arg)
  {
    this->updated_pairs = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    knowledge_server::msg::Update_<ContainerAllocator> *;
  using ConstRawPtr =
    const knowledge_server::msg::Update_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<knowledge_server::msg::Update_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<knowledge_server::msg::Update_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      knowledge_server::msg::Update_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<knowledge_server::msg::Update_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      knowledge_server::msg::Update_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<knowledge_server::msg::Update_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<knowledge_server::msg::Update_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<knowledge_server::msg::Update_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__knowledge_server__msg__Update
    std::shared_ptr<knowledge_server::msg::Update_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__knowledge_server__msg__Update
    std::shared_ptr<knowledge_server::msg::Update_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Update_ & other) const
  {
    if (this->updated_pairs != other.updated_pairs) {
      return false;
    }
    return true;
  }
  bool operator!=(const Update_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Update_

// alias to use template instance with default allocator
using Update =
  knowledge_server::msg::Update_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace knowledge_server

#endif  // KNOWLEDGE_SERVER__MSG__DETAIL__UPDATE__STRUCT_HPP_
