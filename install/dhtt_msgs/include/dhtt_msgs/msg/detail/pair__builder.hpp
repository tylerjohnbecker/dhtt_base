// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dhtt_msgs:msg/Pair.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__MSG__DETAIL__PAIR__BUILDER_HPP_
#define DHTT_MSGS__MSG__DETAIL__PAIR__BUILDER_HPP_

#include "dhtt_msgs/msg/detail/pair__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace dhtt_msgs
{

namespace msg
{

namespace builder
{

class Init_Pair_value
{
public:
  explicit Init_Pair_value(::dhtt_msgs::msg::Pair & msg)
  : msg_(msg)
  {}
  ::dhtt_msgs::msg::Pair value(::dhtt_msgs::msg::Pair::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::msg::Pair msg_;
};

class Init_Pair_key
{
public:
  Init_Pair_key()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Pair_value key(::dhtt_msgs::msg::Pair::_key_type arg)
  {
    msg_.key = std::move(arg);
    return Init_Pair_value(msg_);
  }

private:
  ::dhtt_msgs::msg::Pair msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::msg::Pair>()
{
  return dhtt_msgs::msg::builder::Init_Pair_key();
}

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__MSG__DETAIL__PAIR__BUILDER_HPP_
