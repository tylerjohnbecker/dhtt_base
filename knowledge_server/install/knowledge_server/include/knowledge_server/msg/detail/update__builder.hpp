// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from knowledge_server:msg/Update.idl
// generated code does not contain a copyright notice

#ifndef KNOWLEDGE_SERVER__MSG__DETAIL__UPDATE__BUILDER_HPP_
#define KNOWLEDGE_SERVER__MSG__DETAIL__UPDATE__BUILDER_HPP_

#include "knowledge_server/msg/detail/update__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace knowledge_server
{

namespace msg
{

namespace builder
{

class Init_Update_updated_pairs
{
public:
  Init_Update_updated_pairs()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::knowledge_server::msg::Update updated_pairs(::knowledge_server::msg::Update::_updated_pairs_type arg)
  {
    msg_.updated_pairs = std::move(arg);
    return std::move(msg_);
  }

private:
  ::knowledge_server::msg::Update msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::knowledge_server::msg::Update>()
{
  return knowledge_server::msg::builder::Init_Update_updated_pairs();
}

}  // namespace knowledge_server

#endif  // KNOWLEDGE_SERVER__MSG__DETAIL__UPDATE__BUILDER_HPP_
