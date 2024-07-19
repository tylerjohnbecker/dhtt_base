// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dhtt_msgs:msg/UpdateInfo.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__MSG__DETAIL__UPDATE_INFO__BUILDER_HPP_
#define DHTT_MSGS__MSG__DETAIL__UPDATE_INFO__BUILDER_HPP_

#include "dhtt_msgs/msg/detail/update_info__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace dhtt_msgs
{

namespace msg
{

namespace builder
{

class Init_UpdateInfo_updates
{
public:
  Init_UpdateInfo_updates()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dhtt_msgs::msg::UpdateInfo updates(::dhtt_msgs::msg::UpdateInfo::_updates_type arg)
  {
    msg_.updates = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::msg::UpdateInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::msg::UpdateInfo>()
{
  return dhtt_msgs::msg::builder::Init_UpdateInfo_updates();
}

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__MSG__DETAIL__UPDATE_INFO__BUILDER_HPP_
