// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dhtt_msgs:srv/InternalModifyRequest.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__SRV__DETAIL__INTERNAL_MODIFY_REQUEST__BUILDER_HPP_
#define DHTT_MSGS__SRV__DETAIL__INTERNAL_MODIFY_REQUEST__BUILDER_HPP_

#include "dhtt_msgs/srv/detail/internal_modify_request__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace dhtt_msgs
{

namespace srv
{

namespace builder
{

class Init_InternalModifyRequest_Request_params
{
public:
  explicit Init_InternalModifyRequest_Request_params(::dhtt_msgs::srv::InternalModifyRequest_Request & msg)
  : msg_(msg)
  {}
  ::dhtt_msgs::srv::InternalModifyRequest_Request params(::dhtt_msgs::srv::InternalModifyRequest_Request::_params_type arg)
  {
    msg_.params = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::srv::InternalModifyRequest_Request msg_;
};

class Init_InternalModifyRequest_Request_plugin_name
{
public:
  explicit Init_InternalModifyRequest_Request_plugin_name(::dhtt_msgs::srv::InternalModifyRequest_Request & msg)
  : msg_(msg)
  {}
  Init_InternalModifyRequest_Request_params plugin_name(::dhtt_msgs::srv::InternalModifyRequest_Request::_plugin_name_type arg)
  {
    msg_.plugin_name = std::move(arg);
    return Init_InternalModifyRequest_Request_params(msg_);
  }

private:
  ::dhtt_msgs::srv::InternalModifyRequest_Request msg_;
};

class Init_InternalModifyRequest_Request_node_name
{
public:
  explicit Init_InternalModifyRequest_Request_node_name(::dhtt_msgs::srv::InternalModifyRequest_Request & msg)
  : msg_(msg)
  {}
  Init_InternalModifyRequest_Request_plugin_name node_name(::dhtt_msgs::srv::InternalModifyRequest_Request::_node_name_type arg)
  {
    msg_.node_name = std::move(arg);
    return Init_InternalModifyRequest_Request_plugin_name(msg_);
  }

private:
  ::dhtt_msgs::srv::InternalModifyRequest_Request msg_;
};

class Init_InternalModifyRequest_Request_type
{
public:
  Init_InternalModifyRequest_Request_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InternalModifyRequest_Request_node_name type(::dhtt_msgs::srv::InternalModifyRequest_Request::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_InternalModifyRequest_Request_node_name(msg_);
  }

private:
  ::dhtt_msgs::srv::InternalModifyRequest_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::srv::InternalModifyRequest_Request>()
{
  return dhtt_msgs::srv::builder::Init_InternalModifyRequest_Request_type();
}

}  // namespace dhtt_msgs


namespace dhtt_msgs
{

namespace srv
{

namespace builder
{

class Init_InternalModifyRequest_Response_error_msg
{
public:
  explicit Init_InternalModifyRequest_Response_error_msg(::dhtt_msgs::srv::InternalModifyRequest_Response & msg)
  : msg_(msg)
  {}
  ::dhtt_msgs::srv::InternalModifyRequest_Response error_msg(::dhtt_msgs::srv::InternalModifyRequest_Response::_error_msg_type arg)
  {
    msg_.error_msg = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::srv::InternalModifyRequest_Response msg_;
};

class Init_InternalModifyRequest_Response_success
{
public:
  Init_InternalModifyRequest_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InternalModifyRequest_Response_error_msg success(::dhtt_msgs::srv::InternalModifyRequest_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_InternalModifyRequest_Response_error_msg(msg_);
  }

private:
  ::dhtt_msgs::srv::InternalModifyRequest_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::srv::InternalModifyRequest_Response>()
{
  return dhtt_msgs::srv::builder::Init_InternalModifyRequest_Response_success();
}

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__SRV__DETAIL__INTERNAL_MODIFY_REQUEST__BUILDER_HPP_
