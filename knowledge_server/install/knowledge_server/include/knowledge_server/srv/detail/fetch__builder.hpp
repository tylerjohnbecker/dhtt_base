// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from knowledge_server:srv/Fetch.idl
// generated code does not contain a copyright notice

#ifndef KNOWLEDGE_SERVER__SRV__DETAIL__FETCH__BUILDER_HPP_
#define KNOWLEDGE_SERVER__SRV__DETAIL__FETCH__BUILDER_HPP_

#include "knowledge_server/srv/detail/fetch__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace knowledge_server
{

namespace srv
{

namespace builder
{

class Init_Fetch_Request_keys
{
public:
  Init_Fetch_Request_keys()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::knowledge_server::srv::Fetch_Request keys(::knowledge_server::srv::Fetch_Request::_keys_type arg)
  {
    msg_.keys = std::move(arg);
    return std::move(msg_);
  }

private:
  ::knowledge_server::srv::Fetch_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::knowledge_server::srv::Fetch_Request>()
{
  return knowledge_server::srv::builder::Init_Fetch_Request_keys();
}

}  // namespace knowledge_server


namespace knowledge_server
{

namespace srv
{

namespace builder
{

class Init_Fetch_Response_pairs
{
public:
  Init_Fetch_Response_pairs()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::knowledge_server::srv::Fetch_Response pairs(::knowledge_server::srv::Fetch_Response::_pairs_type arg)
  {
    msg_.pairs = std::move(arg);
    return std::move(msg_);
  }

private:
  ::knowledge_server::srv::Fetch_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::knowledge_server::srv::Fetch_Response>()
{
  return knowledge_server::srv::builder::Init_Fetch_Response_pairs();
}

}  // namespace knowledge_server

#endif  // KNOWLEDGE_SERVER__SRV__DETAIL__FETCH__BUILDER_HPP_
