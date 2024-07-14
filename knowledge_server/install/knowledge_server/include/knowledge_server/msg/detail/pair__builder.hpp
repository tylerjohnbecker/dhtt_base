// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from knowledge_server:msg/Pair.idl
// generated code does not contain a copyright notice

#ifndef KNOWLEDGE_SERVER__MSG__DETAIL__PAIR__BUILDER_HPP_
#define KNOWLEDGE_SERVER__MSG__DETAIL__PAIR__BUILDER_HPP_

#include "knowledge_server/msg/detail/pair__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace knowledge_server
{

namespace msg
{

namespace builder
{

class Init_Pair_value
{
public:
  explicit Init_Pair_value(::knowledge_server::msg::Pair & msg)
  : msg_(msg)
  {}
  ::knowledge_server::msg::Pair value(::knowledge_server::msg::Pair::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::knowledge_server::msg::Pair msg_;
};

class Init_Pair_key
{
public:
  Init_Pair_key()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Pair_value key(::knowledge_server::msg::Pair::_key_type arg)
  {
    msg_.key = std::move(arg);
    return Init_Pair_value(msg_);
  }

private:
  ::knowledge_server::msg::Pair msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::knowledge_server::msg::Pair>()
{
  return knowledge_server::msg::builder::Init_Pair_key();
}

}  // namespace knowledge_server

#endif  // KNOWLEDGE_SERVER__MSG__DETAIL__PAIR__BUILDER_HPP_
