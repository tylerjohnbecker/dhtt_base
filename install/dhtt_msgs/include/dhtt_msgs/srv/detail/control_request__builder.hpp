// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dhtt_msgs:srv/ControlRequest.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__SRV__DETAIL__CONTROL_REQUEST__BUILDER_HPP_
#define DHTT_MSGS__SRV__DETAIL__CONTROL_REQUEST__BUILDER_HPP_

#include "dhtt_msgs/srv/detail/control_request__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace dhtt_msgs
{

namespace srv
{

namespace builder
{

class Init_ControlRequest_Request_interrupt
{
public:
  explicit Init_ControlRequest_Request_interrupt(::dhtt_msgs::srv::ControlRequest_Request & msg)
  : msg_(msg)
  {}
  ::dhtt_msgs::srv::ControlRequest_Request interrupt(::dhtt_msgs::srv::ControlRequest_Request::_interrupt_type arg)
  {
    msg_.interrupt = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::srv::ControlRequest_Request msg_;
};

class Init_ControlRequest_Request_file_name
{
public:
  explicit Init_ControlRequest_Request_file_name(::dhtt_msgs::srv::ControlRequest_Request & msg)
  : msg_(msg)
  {}
  Init_ControlRequest_Request_interrupt file_name(::dhtt_msgs::srv::ControlRequest_Request::_file_name_type arg)
  {
    msg_.file_name = std::move(arg);
    return Init_ControlRequest_Request_interrupt(msg_);
  }

private:
  ::dhtt_msgs::srv::ControlRequest_Request msg_;
};

class Init_ControlRequest_Request_type
{
public:
  Init_ControlRequest_Request_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ControlRequest_Request_file_name type(::dhtt_msgs::srv::ControlRequest_Request::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_ControlRequest_Request_file_name(msg_);
  }

private:
  ::dhtt_msgs::srv::ControlRequest_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::srv::ControlRequest_Request>()
{
  return dhtt_msgs::srv::builder::Init_ControlRequest_Request_type();
}

}  // namespace dhtt_msgs


namespace dhtt_msgs
{

namespace srv
{

namespace builder
{

class Init_ControlRequest_Response_error_msg
{
public:
  explicit Init_ControlRequest_Response_error_msg(::dhtt_msgs::srv::ControlRequest_Response & msg)
  : msg_(msg)
  {}
  ::dhtt_msgs::srv::ControlRequest_Response error_msg(::dhtt_msgs::srv::ControlRequest_Response::_error_msg_type arg)
  {
    msg_.error_msg = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::srv::ControlRequest_Response msg_;
};

class Init_ControlRequest_Response_success
{
public:
  Init_ControlRequest_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ControlRequest_Response_error_msg success(::dhtt_msgs::srv::ControlRequest_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ControlRequest_Response_error_msg(msg_);
  }

private:
  ::dhtt_msgs::srv::ControlRequest_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::srv::ControlRequest_Response>()
{
  return dhtt_msgs::srv::builder::Init_ControlRequest_Response_success();
}

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__SRV__DETAIL__CONTROL_REQUEST__BUILDER_HPP_
