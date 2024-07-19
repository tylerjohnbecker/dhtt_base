// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dhtt_msgs:msg/Resource.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__MSG__DETAIL__RESOURCE__BUILDER_HPP_
#define DHTT_MSGS__MSG__DETAIL__RESOURCE__BUILDER_HPP_

#include "dhtt_msgs/msg/detail/resource__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace dhtt_msgs
{

namespace msg
{

namespace builder
{

class Init_Resource_channel
{
public:
  explicit Init_Resource_channel(::dhtt_msgs::msg::Resource & msg)
  : msg_(msg)
  {}
  ::dhtt_msgs::msg::Resource channel(::dhtt_msgs::msg::Resource::_channel_type arg)
  {
    msg_.channel = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::msg::Resource msg_;
};

class Init_Resource_name
{
public:
  explicit Init_Resource_name(::dhtt_msgs::msg::Resource & msg)
  : msg_(msg)
  {}
  Init_Resource_channel name(::dhtt_msgs::msg::Resource::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_Resource_channel(msg_);
  }

private:
  ::dhtt_msgs::msg::Resource msg_;
};

class Init_Resource_owners
{
public:
  explicit Init_Resource_owners(::dhtt_msgs::msg::Resource & msg)
  : msg_(msg)
  {}
  Init_Resource_name owners(::dhtt_msgs::msg::Resource::_owners_type arg)
  {
    msg_.owners = std::move(arg);
    return Init_Resource_name(msg_);
  }

private:
  ::dhtt_msgs::msg::Resource msg_;
};

class Init_Resource_locked
{
public:
  explicit Init_Resource_locked(::dhtt_msgs::msg::Resource & msg)
  : msg_(msg)
  {}
  Init_Resource_owners locked(::dhtt_msgs::msg::Resource::_locked_type arg)
  {
    msg_.locked = std::move(arg);
    return Init_Resource_owners(msg_);
  }

private:
  ::dhtt_msgs::msg::Resource msg_;
};

class Init_Resource_type
{
public:
  Init_Resource_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Resource_locked type(::dhtt_msgs::msg::Resource::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_Resource_locked(msg_);
  }

private:
  ::dhtt_msgs::msg::Resource msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::msg::Resource>()
{
  return dhtt_msgs::msg::builder::Init_Resource_type();
}

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__MSG__DETAIL__RESOURCE__BUILDER_HPP_
