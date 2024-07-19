// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dhtt_msgs:msg/NodeStatus.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__MSG__DETAIL__NODE_STATUS__BUILDER_HPP_
#define DHTT_MSGS__MSG__DETAIL__NODE_STATUS__BUILDER_HPP_

#include "dhtt_msgs/msg/detail/node_status__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace dhtt_msgs
{

namespace msg
{

namespace builder
{

class Init_NodeStatus_state
{
public:
  Init_NodeStatus_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dhtt_msgs::msg::NodeStatus state(::dhtt_msgs::msg::NodeStatus::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::msg::NodeStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::msg::NodeStatus>()
{
  return dhtt_msgs::msg::builder::Init_NodeStatus_state();
}

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__MSG__DETAIL__NODE_STATUS__BUILDER_HPP_
