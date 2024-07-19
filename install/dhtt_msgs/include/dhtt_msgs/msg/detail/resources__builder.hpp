// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dhtt_msgs:msg/Resources.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__MSG__DETAIL__RESOURCES__BUILDER_HPP_
#define DHTT_MSGS__MSG__DETAIL__RESOURCES__BUILDER_HPP_

#include "dhtt_msgs/msg/detail/resources__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace dhtt_msgs
{

namespace msg
{

namespace builder
{

class Init_Resources_resource_state
{
public:
  Init_Resources_resource_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dhtt_msgs::msg::Resources resource_state(::dhtt_msgs::msg::Resources::_resource_state_type arg)
  {
    msg_.resource_state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::msg::Resources msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::msg::Resources>()
{
  return dhtt_msgs::msg::builder::Init_Resources_resource_state();
}

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__MSG__DETAIL__RESOURCES__BUILDER_HPP_
