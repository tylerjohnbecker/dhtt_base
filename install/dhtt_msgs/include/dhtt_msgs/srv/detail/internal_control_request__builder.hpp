// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dhtt_msgs:srv/InternalControlRequest.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__SRV__DETAIL__INTERNAL_CONTROL_REQUEST__BUILDER_HPP_
#define DHTT_MSGS__SRV__DETAIL__INTERNAL_CONTROL_REQUEST__BUILDER_HPP_

#include "dhtt_msgs/srv/detail/internal_control_request__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace dhtt_msgs
{

namespace srv
{

namespace builder
{

class Init_InternalControlRequest_Request_control_code
{
public:
  Init_InternalControlRequest_Request_control_code()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dhtt_msgs::srv::InternalControlRequest_Request control_code(::dhtt_msgs::srv::InternalControlRequest_Request::_control_code_type arg)
  {
    msg_.control_code = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::srv::InternalControlRequest_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::srv::InternalControlRequest_Request>()
{
  return dhtt_msgs::srv::builder::Init_InternalControlRequest_Request_control_code();
}

}  // namespace dhtt_msgs


namespace dhtt_msgs
{

namespace srv
{

namespace builder
{

class Init_InternalControlRequest_Response_error_msg
{
public:
  explicit Init_InternalControlRequest_Response_error_msg(::dhtt_msgs::srv::InternalControlRequest_Response & msg)
  : msg_(msg)
  {}
  ::dhtt_msgs::srv::InternalControlRequest_Response error_msg(::dhtt_msgs::srv::InternalControlRequest_Response::_error_msg_type arg)
  {
    msg_.error_msg = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::srv::InternalControlRequest_Response msg_;
};

class Init_InternalControlRequest_Response_success
{
public:
  Init_InternalControlRequest_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InternalControlRequest_Response_error_msg success(::dhtt_msgs::srv::InternalControlRequest_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_InternalControlRequest_Response_error_msg(msg_);
  }

private:
  ::dhtt_msgs::srv::InternalControlRequest_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::srv::InternalControlRequest_Response>()
{
  return dhtt_msgs::srv::builder::Init_InternalControlRequest_Response_success();
}

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__SRV__DETAIL__INTERNAL_CONTROL_REQUEST__BUILDER_HPP_
