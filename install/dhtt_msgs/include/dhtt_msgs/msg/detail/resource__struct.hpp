// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dhtt_msgs:msg/Resource.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__MSG__DETAIL__RESOURCE__STRUCT_HPP_
#define DHTT_MSGS__MSG__DETAIL__RESOURCE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__dhtt_msgs__msg__Resource __attribute__((deprecated))
#else
# define DEPRECATED__dhtt_msgs__msg__Resource __declspec(deprecated)
#endif

namespace dhtt_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Resource_
{
  using Type = Resource_<ContainerAllocator>;

  explicit Resource_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0;
      this->locked = false;
      this->owners = 0;
      this->name = "";
      this->channel = 0;
    }
  }

  explicit Resource_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0;
      this->locked = false;
      this->owners = 0;
      this->name = "";
      this->channel = 0;
    }
  }

  // field types and members
  using _type_type =
    int8_t;
  _type_type type;
  using _locked_type =
    bool;
  _locked_type locked;
  using _owners_type =
    int16_t;
  _owners_type owners;
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _name_type name;
  using _channel_type =
    int8_t;
  _channel_type channel;

  // setters for named parameter idiom
  Type & set__type(
    const int8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__locked(
    const bool & _arg)
  {
    this->locked = _arg;
    return *this;
  }
  Type & set__owners(
    const int16_t & _arg)
  {
    this->owners = _arg;
    return *this;
  }
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__channel(
    const int8_t & _arg)
  {
    this->channel = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int8_t GRIPPER =
    1;
  static constexpr int8_t HEAD =
    2;
  static constexpr int8_t BASE =
    3;
  static constexpr int8_t EXCLUSIVE =
    1;
  static constexpr int8_t SHARED =
    2;

  // pointer types
  using RawPtr =
    dhtt_msgs::msg::Resource_<ContainerAllocator> *;
  using ConstRawPtr =
    const dhtt_msgs::msg::Resource_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dhtt_msgs::msg::Resource_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dhtt_msgs::msg::Resource_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::msg::Resource_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::msg::Resource_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::msg::Resource_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::msg::Resource_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dhtt_msgs::msg::Resource_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dhtt_msgs::msg::Resource_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dhtt_msgs__msg__Resource
    std::shared_ptr<dhtt_msgs::msg::Resource_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dhtt_msgs__msg__Resource
    std::shared_ptr<dhtt_msgs::msg::Resource_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Resource_ & other) const
  {
    if (this->type != other.type) {
      return false;
    }
    if (this->locked != other.locked) {
      return false;
    }
    if (this->owners != other.owners) {
      return false;
    }
    if (this->name != other.name) {
      return false;
    }
    if (this->channel != other.channel) {
      return false;
    }
    return true;
  }
  bool operator!=(const Resource_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Resource_

// alias to use template instance with default allocator
using Resource =
  dhtt_msgs::msg::Resource_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr int8_t Resource_<ContainerAllocator>::GRIPPER;
template<typename ContainerAllocator>
constexpr int8_t Resource_<ContainerAllocator>::HEAD;
template<typename ContainerAllocator>
constexpr int8_t Resource_<ContainerAllocator>::BASE;
template<typename ContainerAllocator>
constexpr int8_t Resource_<ContainerAllocator>::EXCLUSIVE;
template<typename ContainerAllocator>
constexpr int8_t Resource_<ContainerAllocator>::SHARED;

}  // namespace msg

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__MSG__DETAIL__RESOURCE__STRUCT_HPP_
