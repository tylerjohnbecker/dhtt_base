// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dhtt_msgs:srv/HistoryRequest.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__SRV__DETAIL__HISTORY_REQUEST__BUILDER_HPP_
#define DHTT_MSGS__SRV__DETAIL__HISTORY_REQUEST__BUILDER_HPP_

#include "dhtt_msgs/srv/detail/history_request__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace dhtt_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::srv::HistoryRequest_Request>()
{
  return ::dhtt_msgs::srv::HistoryRequest_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace dhtt_msgs


namespace dhtt_msgs
{

namespace srv
{

namespace builder
{

class Init_HistoryRequest_Response_node_history
{
public:
  Init_HistoryRequest_Response_node_history()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dhtt_msgs::srv::HistoryRequest_Response node_history(::dhtt_msgs::srv::HistoryRequest_Response::_node_history_type arg)
  {
    msg_.node_history = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::srv::HistoryRequest_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::srv::HistoryRequest_Response>()
{
  return dhtt_msgs::srv::builder::Init_HistoryRequest_Response_node_history();
}

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__SRV__DETAIL__HISTORY_REQUEST__BUILDER_HPP_
