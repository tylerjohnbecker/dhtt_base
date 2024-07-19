// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dhtt_msgs:srv/FetchInfo.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__SRV__DETAIL__FETCH_INFO__BUILDER_HPP_
#define DHTT_MSGS__SRV__DETAIL__FETCH_INFO__BUILDER_HPP_

#include "dhtt_msgs/srv/detail/fetch_info__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace dhtt_msgs
{

namespace srv
{

namespace builder
{

class Init_FetchInfo_Request_keys
{
public:
  Init_FetchInfo_Request_keys()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dhtt_msgs::srv::FetchInfo_Request keys(::dhtt_msgs::srv::FetchInfo_Request::_keys_type arg)
  {
    msg_.keys = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::srv::FetchInfo_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::srv::FetchInfo_Request>()
{
  return dhtt_msgs::srv::builder::Init_FetchInfo_Request_keys();
}

}  // namespace dhtt_msgs


namespace dhtt_msgs
{

namespace srv
{

namespace builder
{

class Init_FetchInfo_Response_information_pairs
{
public:
  Init_FetchInfo_Response_information_pairs()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dhtt_msgs::srv::FetchInfo_Response information_pairs(::dhtt_msgs::srv::FetchInfo_Response::_information_pairs_type arg)
  {
    msg_.information_pairs = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::srv::FetchInfo_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::srv::FetchInfo_Response>()
{
  return dhtt_msgs::srv::builder::Init_FetchInfo_Response_information_pairs();
}

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__SRV__DETAIL__FETCH_INFO__BUILDER_HPP_
