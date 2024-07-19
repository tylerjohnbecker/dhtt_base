// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dhtt_msgs:srv/GoitrRequest.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__SRV__DETAIL__GOITR_REQUEST__BUILDER_HPP_
#define DHTT_MSGS__SRV__DETAIL__GOITR_REQUEST__BUILDER_HPP_

#include "dhtt_msgs/srv/detail/goitr_request__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace dhtt_msgs
{

namespace srv
{

namespace builder
{

class Init_GoitrRequest_Request_params
{
public:
  explicit Init_GoitrRequest_Request_params(::dhtt_msgs::srv::GoitrRequest_Request & msg)
  : msg_(msg)
  {}
  ::dhtt_msgs::srv::GoitrRequest_Request params(::dhtt_msgs::srv::GoitrRequest_Request::_params_type arg)
  {
    msg_.params = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::srv::GoitrRequest_Request msg_;
};

class Init_GoitrRequest_Request_type
{
public:
  Init_GoitrRequest_Request_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoitrRequest_Request_params type(::dhtt_msgs::srv::GoitrRequest_Request::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_GoitrRequest_Request_params(msg_);
  }

private:
  ::dhtt_msgs::srv::GoitrRequest_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::srv::GoitrRequest_Request>()
{
  return dhtt_msgs::srv::builder::Init_GoitrRequest_Request_type();
}

}  // namespace dhtt_msgs


namespace dhtt_msgs
{

namespace srv
{

namespace builder
{

class Init_GoitrRequest_Response_responses
{
public:
  explicit Init_GoitrRequest_Response_responses(::dhtt_msgs::srv::GoitrRequest_Response & msg)
  : msg_(msg)
  {}
  ::dhtt_msgs::srv::GoitrRequest_Response responses(::dhtt_msgs::srv::GoitrRequest_Response::_responses_type arg)
  {
    msg_.responses = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::srv::GoitrRequest_Response msg_;
};

class Init_GoitrRequest_Response_error_msgs
{
public:
  explicit Init_GoitrRequest_Response_error_msgs(::dhtt_msgs::srv::GoitrRequest_Response & msg)
  : msg_(msg)
  {}
  Init_GoitrRequest_Response_responses error_msgs(::dhtt_msgs::srv::GoitrRequest_Response::_error_msgs_type arg)
  {
    msg_.error_msgs = std::move(arg);
    return Init_GoitrRequest_Response_responses(msg_);
  }

private:
  ::dhtt_msgs::srv::GoitrRequest_Response msg_;
};

class Init_GoitrRequest_Response_success
{
public:
  Init_GoitrRequest_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoitrRequest_Response_error_msgs success(::dhtt_msgs::srv::GoitrRequest_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GoitrRequest_Response_error_msgs(msg_);
  }

private:
  ::dhtt_msgs::srv::GoitrRequest_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::srv::GoitrRequest_Response>()
{
  return dhtt_msgs::srv::builder::Init_GoitrRequest_Response_success();
}

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__SRV__DETAIL__GOITR_REQUEST__BUILDER_HPP_
